// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <algorithm> // std::any_of, std::find_if
#include <iterator> // std::distance

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include <yarp/dev/IAxisInfo.h>

#include "ICanBusSharer.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

const yarp::dev::PolyDriverDescriptor * CanBusBroker::tryCreateFakeNode(const yarp::dev::PolyDriverDescriptor * driver)
{
    for (auto i = 0; i < fakeNodes.size(); i++)
    {
        if (fakeNodes[i]->key == driver->poly->id())
        {
            yCInfo(CBB) << "Requested instantiation of fake node device" << driver->key;

            yarp::dev::IAxisInfoRaw * iAxisInfo;

            if (!driver->poly->view(iAxisInfo))
            {
                yCError(CBB) << "Unable to view IAxisInfoRaw in" << driver->key;
                return nullptr;
            }

            int axes;

            if (!iAxisInfo->getAxes(&axes))
            {
                yCError(CBB) << "Unable to get number of axes in" << driver->key;
                return nullptr;
            }

            yarp::os::Bottle axisNames;
            yarp::os::Bottle jointTypes;

            for (int j = 0; j < axes; j++)
            {
                std::string axisName;
                yarp::dev::JointTypeEnum jointType;

                if (!iAxisInfo->getAxisNameRaw(j, axisName))
                {
                    yCError(CBB) << "Unable to get axis name for joint" << j << "in" << driver->key;
                    return nullptr;
                }

                if (!iAxisInfo->getJointTypeRaw(j, jointType))
                {
                    yCError(CBB) << "Unable to get joint type for joint" << j << "in" << driver->key;
                    return nullptr;
                }

                axisNames.addString(axisName);
                jointTypes.addVocab32(jointType);
            }

            yarp::os::Property options {
                {"device", yarp::os::Value("FakeJoint")},
                {"axes", yarp::os::Value(axes)}
            };

            // no initializer list overload for these, sadly
            options.put("axisNames", yarp::os::Value::makeList(axisNames.toString().c_str()));
            options.put("jointTypes", yarp::os::Value::makeList(jointTypes.toString().c_str()));

            auto * fakeNode = new yarp::dev::PolyDriver;
            fakeNodes[i]->poly = fakeNode;

            fakeNode->setId(driver->poly->id());

            if (!fakeNode->open(options))
            {
                yCError(CBB) << "Unable to open fake node device" << fakeNodes[i]->key;
                return nullptr;
            }

            fakeNodes[i]->key = driver->key; // hackish, but we need this later on during attach
            return fakeNodes[i];
        }
    }

    return driver;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::attachAll(const yarp::dev::PolyDriverList & drivers)
{
    const auto & nodeNames = broker->getNodeNames();
    const yarp::dev::PolyDriverDescriptor * bus = nullptr;
    std::vector<const yarp::dev::PolyDriverDescriptor *> nodes(nodeNames.size());

    for (int i = 0; i < drivers.size(); i++)
    {
        const auto * driver = drivers[i];

        if (driver->key == broker->getBusName())
        {
            if (bus != nullptr)
            {
                yCError(CBB) << "Bus device" << driver->key << "already attached";
                return false;
            }

            bus = driver;
        }
        else if (auto node = std::find_if(nodeNames.begin(), nodeNames.end(), [driver](const auto & name)
                                                                              { return name == driver->key; });
                 node != nodeNames.end())
        {
            int index = std::distance(nodeNames.begin(), node);

            if (nodes[index] != nullptr)
            {
                yCError(CBB) << "Node device" << driver->key << "already attached";
                return false;
            }

            nodes[index] = tryCreateFakeNode(driver);
        }
        else
        {
            yCError(CBB) << "Unknown device:" << driver->key;
            return false;
        }
    }

    if (bus == nullptr)
    {
        yCError(CBB) << "The bus device is missing:" << broker->getBusName();
        return false;
    }

    if (std::any_of(nodes.begin(), nodes.end(), [](const auto * node) { return node == nullptr; }))
    {
        std::vector<std::string> names;

        for (int i = 0; i < nodes.size(); i++)
        {
            if (nodes[i] == nullptr)
            {
                names.push_back(nodeNames[i]);
            }
        }

        yCError(CBB) << "Some node devices are missing:" << names;
        return false;
    }

    for (int i = 0; i < fakeNodes.size(); i++)
    {
        if (fakeNodes[i]->poly == nullptr)
        {
            yCError(CBB) << "Fake node device" << fakeNodes[i]->key << "requested, but not instantiated";
            return false;
        }
    }

    yCInfo(CBB) << "Attached" << bus->key << "bus device and" << nodes.size() << "node device(s):" << nodeNames;

    if (!broker->registerDevice(bus->poly))
    {
        yCError(CBB) << "Unable to register bus device" << bus->key;
        return false;
    }

    yCInfo(CBB) << "Registered bus device";

    for (int i = 0; i < nodes.size(); i++)
    {
        const auto * node = nodes[i];

        if (!deviceMapper.registerDevice(node->poly))
        {
            yCError(CBB) << "Unable to register node device" << node->key;
            return false;
        }

        ICanBusSharer * iCanBusSharer;

        if (!node->poly->view(iCanBusSharer))
        {
            yCError(CBB) << "Unable to view ICanBusSharer in" << node->key;
            return false;
        }

        broker->getReader()->registerHandle(iCanBusSharer);
        iCanBusSharer->registerSender(broker->getWriter()->getDelegate());
    }

    yCInfo(CBB) << "Registered node devices";

    if (!broker->startThreads())
    {
        yCError(CBB) << "Unable to start CAN threads in" << broker->getBusName();
        return false;
    }

    yCInfo(CBB) << "Started CAN threads";

    for (const auto & rawDevice : deviceMapper.getDevices())
    {
        auto * iCanBusSharer = rawDevice->castToType<ICanBusSharer>();

        if (iCanBusSharer && !iCanBusSharer->initialize())
        {
            yCWarning(CBB) << "Node device id" << iCanBusSharer->getId() << "could not initialize CAN comms";
        }
    }

    broker->markInitialized(true);

    yCInfo(CBB) << "Initialized node devices";

    if (auto & syncThread = getSyncThread(); !syncThread.getBrokers().empty())
    {
        if (!syncThread.openPort("/sync:o"))
        {
            yCError(CBB) << "Unable to open synchronization port";
            return false;
        }

        if (!syncThread.isRunning())
        {
            if (!syncThread.start())
            {
                yCError(CBB) << "Unable to start synchronization thread";
                return false;
            }
            else
            {
                yCInfo(CBB) << "Started synchronization thread";
            }
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::detachAll()
{
    bool ok = true;

    if (broker)
    {
        broker->markInitialized(false);
    }

    if (auto & syncThread = getSyncThread(); syncThread.isRunning())
    {
        syncThread.stop();
        syncThread.closePort();

        yCInfo(CBB) << "Stopped synchronization thread";
    }

    for (const auto & rawDevice : deviceMapper.getDevices())
    {
        auto * iCanBusSharer = rawDevice->castToType<ICanBusSharer>();
        yCDebug(CBB) << "Finalizing node device id" << iCanBusSharer->getId();

        if (iCanBusSharer && !iCanBusSharer->finalize())
        {
            yCWarning(CBB) << "Node device id" << iCanBusSharer->getId() << "could not finalize CAN comms";
            ok = false;
        }

        yCDebug(CBB) << "Unregistering node device id" << iCanBusSharer->getId();
    }

    deviceMapper.clear();

    if (broker)
    {
        ok &= broker->stopThreads();
        ok &= broker->clearFilters();
    }

    yCDebug(CBB) << "Stopped CAN threads";

    for (int i = 0; i < fakeNodes.size(); i++)
    {
        if (fakeNodes[i]->poly && fakeNodes[i]->poly->isValid())
        {
            ok &= fakeNodes[i]->poly->close();
            delete fakeNodes[i]->poly;
            fakeNodes[i]->poly = nullptr;
        }
    }

    return ok;
}

// -----------------------------------------------------------------------------
