// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <algorithm> // std::any_of, std::find_if, std::for_each
#include <iterator> // std::distance

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
        if (fakeNodes[i]->key == driver->key)
        {
            yCInfo(CBB) << "Requested instantiation of fake node device" << driver->key;

            yarp::dev::IAxisInfoRaw * iAxisInfo;

            if (!driver->poly->view(iAxisInfo))
            {
                yCError(CBB) << "Unable to view IAxisInfoRaw in" << driver->key;
                return nullptr;
            }

            std::string axisName;

            if (!iAxisInfo->getAxisNameRaw(0, axisName))
            {
                yCError(CBB) << "Unable to get axis name in" << driver->key;
                return nullptr;
            }

            yarp::dev::JointTypeEnum jointType;

            if (!iAxisInfo->getJointTypeRaw(0, jointType))
            {
                yCError(CBB) << "Unable to get joint type in" << driver->key;
                return nullptr;
            }

            yarp::os::Property options {
                {"device", yarp::os::Value("FakeJoint")},
                {"axisName", yarp::os::Value(axisName)},
                {"jointType", yarp::os::Value(jointType, true)} // isVocab32 = true
            };

            auto * fakeNode = new yarp::dev::PolyDriver;
            fakeNodes[i]->poly = fakeNode;

            if (!fakeNode->open(options))
            {
                yCError(CBB) << "Unable to open fake node device" << driver->key;
                return nullptr;
            }

            return fakeNodes[i];
        }
    }

    return driver;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::attachAll(const yarp::dev::PolyDriverList & drivers)
{
    std::vector<std::string> nodeNames; // flattened broker[i]->getNodeNames()

    std::for_each(brokers.begin(), brokers.end(), [&nodeNames](const auto * broker)
                                                  { nodeNames.insert(nodeNames.end(),
                                                                     broker->getNodeNames().begin(),
                                                                     broker->getNodeNames().end()); });

    std::vector<const yarp::dev::PolyDriverDescriptor *> buses(brokers.size());
    std::vector<const yarp::dev::PolyDriverDescriptor *> nodes(nodeNames.size());

    bool hasBusDevices = false;
    bool hasNodeDevices = false;

    for (int i = 0; i < drivers.size(); i++)
    {
        const auto * driver = drivers[i];

        if (auto bus = std::find_if(brokers.begin(), brokers.end(), [driver](const auto * broker)
                                                                    { return broker->getBusName() == driver->key; });
            bus != brokers.end())
        {
            int index = std::distance(brokers.begin(), bus);

            if (buses[index] != nullptr)
            {
                yCError(CBB) << "Bus device" << driver->key << "already attached";
                return false;
            }

            buses[index] = driver;
            hasBusDevices = true;
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
            hasNodeDevices = true;
        }
        else
        {
            yCError(CBB) << "Unknown device:" << driver->key;
            return false;
        }
    }

    if (hasBusDevices && std::any_of(buses.begin(), buses.end(), [](const auto * bus) { return bus == nullptr; }))
    {
        std::vector<std::string> names;

        for (int i = 0; i < buses.size(); i++)
        {
            if (buses[i] == nullptr)
            {
                names.push_back(brokers[i]->getBusName());
            }
        }

        yCError(CBB) << "Some bus devices are missing:" << names;
        return false;
    }

    if (hasNodeDevices && std::any_of(nodes.begin(), nodes.end(), [](const auto * node) { return node == nullptr; }))
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

    if (hasNodeDevices && std::any_of(brokers.begin(), brokers.end(), [](const auto * broker) { return !broker->isRegistered(); }))
    {
        yCError(CBB) << "Bus devices need to be attached prior to node devices";
        return false;
    }

    for (int i = 0; i < buses.size(); i++)
    {
        if (!brokers[i]->registerDevice(buses[i]->poly))
        {
            yCError(CBB) << "Unable to register bus device" << buses[i]->key;
            return false;
        }

        if (!brokers[i]->startThreads())
        {
            yCError(CBB) << "Unable to start CAN threads in" << brokers[i]->getBusName();
            return false;
        }
    }

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

        auto it = std::find_if(brokers.begin(), brokers.end(), [node](const auto * broker)
                                                               { const auto & names = broker->getNodeNames();
                                                                 return std::find(names.begin(), names.end(), node->key) != names.end(); });

        auto * broker = *it;

        broker->getReader()->registerHandle(iCanBusSharer);
        iCanBusSharer->registerSender(broker->getWriter()->getDelegate());
    }

    for (const auto & rawDevice : deviceMapper.getDevices())
    {
        auto * iCanBusSharer = rawDevice->castToType<ICanBusSharer>();

        if (iCanBusSharer && !iCanBusSharer->initialize())
        {
            yCError(CBB) << "Node device id" << iCanBusSharer->getId() << "could not initialize CAN comms";
        }
    }

    return !syncThread || syncThread->start();
}

// -----------------------------------------------------------------------------

bool CanBusBroker::detachAll()
{
    bool ok = true;

    if (syncThread && syncThread->isRunning())
    {
        syncThread->stop();
    }

    delete syncThread;
    syncThread = nullptr;

    for (const auto & rawDevice : deviceMapper.getDevices())
    {
        auto * iCanBusSharer = rawDevice->castToType<ICanBusSharer>();

        if (iCanBusSharer && !iCanBusSharer->finalize())
        {
            yCWarning(CBB) << "Node device id" << iCanBusSharer->getId() << "could not finalize CAN comms";
            ok = false;
        }
    }

    deviceMapper.clear();

    for (auto * broker : brokers)
    {
        ok &= broker->stopThreads();
        ok &= broker->clearFilters();
    }

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
