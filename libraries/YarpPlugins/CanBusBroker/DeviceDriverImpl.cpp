// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include "ICanBusSharer.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::open(yarp::os::Searchable & config)
{
    if (!config.check("robotConfig") || !config.find("robotConfig").isBlob())
    {
        yCError(CBB) << "Missing \"robotConfig\" property or not a blob";
        return false;
    }

    const auto * robotConfig = *reinterpret_cast<yarp::os::Property * const *>(config.find("robotConfig").asBlob());

    const auto * canBuses = config.find("buses").asList();

    if (!canBuses)
    {
        yCError(CBB) << "Missing key \"buses\" or not a list";
        return false;
    }

    for (int i = 0; i < canBuses->size(); i++)
    {
        auto canBus = canBuses->get(i).asString();

        if (!config.check(canBus))
        {
            yCError(CBB) << "Missing CAN bus key:" << canBus;
            return false;
        }

        const auto & nodesVal = config.find(canBus);

        if (!nodesVal.isList())
        {
            yCError(CBB) << "Key" << canBus << "must be a list";
            return false;
        }

        const auto * nodes = nodesVal.asList();
        auto isFakeBus = canBus.rfind("fake-", 0) == 0; // starts with "fake-"?
        std::vector<ICanBusSharer *> busSharers;
        yarp::os::Bottle nodeIds;

        for (int j = 0; j < nodes->size(); j++)
        {
            auto node = nodes->get(j).asString();
            auto isFakeNode = node.rfind("fake-", 0) == 0; // starts with "fake-"?

            yarp::os::Property nodeOptions;

            if (!isFakeNode)
            {
                const auto & nodeGroup = robotConfig->findGroup(node);

                if (nodeGroup.isNull())
                {
                    yCError(CBB) << "Missing CAN node device group" << node;
                    return false;
                }

                nodeOptions.fromString(nodeGroup.toString());
                nodeOptions.put("robotConfig", config.find("robotConfig"));

                if (config.check("syncPeriod"))
                {
                    nodeOptions.put("syncPeriod", config.find("syncPeriod"));
                }
            }
            else
            {
                nodeOptions.put("device", "FakeJoint");
                nodeOptions.put("jointName", node.substr(5));
            }

            auto * device = new yarp::dev::PolyDriver;
            nodeDevices.push_back(device);

            if (!device->open(nodeOptions))
            {
                yCError(CBB) << "CAN node device" << node << "configuration failure";
                return false;
            }

            if (!deviceMapper.registerDevice(device))
            {
                yCError(CBB) << "Unable to register CAN node device" << node;
                return false;
            }

            if (!isFakeBus && !isFakeNode)
            {
                ICanBusSharer * iCanBusSharer;

                if (!device->view(iCanBusSharer))
                {
                    yCError(CBB) << "Unable to view ICanBusSharer in" << node;
                    return false;
                }

                busSharers.push_back(iCanBusSharer);
                nodeIds.addInt32(iCanBusSharer->getId());

                for (auto id : iCanBusSharer->getAdditionalIds())
                {
                    nodeIds.addInt32(id);
                }
            }
        }

        if (!isFakeBus)
        {
            const auto & canBusGroup = robotConfig->findGroup(canBus);

            if (canBusGroup.isNull())
            {
                yCError(CBB) << "Missing CAN bus device group" << canBus;
                return false;
            }

            yarp::os::Property canBusOptions;
            canBusOptions.fromString(canBusGroup.toString());
            canBusOptions.put("robotConfig", config.find("robotConfig"));
            canBusOptions.put("blockingMode", false); // enforce non-blocking mode
            canBusOptions.put("allowPermissive", false); // always check usage requirements

            auto enableAcceptanceFilters = canBusOptions.check("enableAcceptanceFilters", yarp::os::Value(false),
                "enable CAN acceptance filters").asBool();

            if (enableAcceptanceFilters && nodeIds.size() != 0)
            {
                canBusOptions.put("filteredIds", yarp::os::Value::makeList(nodeIds.toString().c_str()));
            }

            auto * canBusDevice = new yarp::dev::PolyDriver;
            busDevices.push_back(canBusDevice);

            if (!canBusDevice->open(canBusOptions))
            {
                yCError(CBB) << "canBusDevice instantiation failed:" << canBus;
                return false;
            }

            auto * broker = new SingleBusBroker(canBus);
            brokers.push_back(broker);

            if (!broker->configure(canBusOptions))
            {
                yCError(CBB) << "Unable to configure broker of CAN bus device" << canBus;
                return false;
            }

            if (!broker->registerDevice(canBusDevice))
            {
                yCError(CBB) << "Unable to register CAN bus device" << canBus;
                return false;
            }

            for (auto * iCanBusSharer : busSharers)
            {
                broker->getReader()->registerHandle(iCanBusSharer);
                iCanBusSharer->registerSender(broker->getWriter()->getDelegate());
            }
        }
    }

    for (auto * broker : brokers)
    {
        if (!broker->startThreads())
        {
            yCError(CBB) << "Unable to start CAN threads in" << broker->getName();
            return false;
        }
    }

    for (const auto & rawDevice : deviceMapper.getDevices())
    {
        auto * iCanBusSharer = rawDevice->castToType<ICanBusSharer>();

        if (iCanBusSharer && !iCanBusSharer->initialize())
        {
            yCError(CBB) << "Node device id" << iCanBusSharer->getId() << "could not initialize CAN comms";
        }
    }

    if (config.check("syncPeriod", "SYNC message period (s)"))
    {
        auto syncPeriod = config.find("syncPeriod").asFloat64();

        if (syncPeriod <= 0.0)
        {
            yCError(CBB) << "Invalid --syncPeriod:" << syncPeriod;
            return false;
        }

        FutureTaskFactory * taskFactory = nullptr;

        if (busDevices.size() > 1)
        {
            taskFactory = new ParallelTaskFactory(busDevices.size());
        }
        else if (busDevices.size() == 1)
        {
            taskFactory = new SequentialTaskFactory;
        }

        if (taskFactory)
        {
            syncThread = new SyncPeriodicThread(brokers, taskFactory); // owns `taskFactory`
            syncThread->setPeriod(syncPeriod);

            if (!syncThread->openPort("/sync:o"))
            {
                yCError(CBB) << "Unable to open sync port";
                return false;
            }

            yarp::os::Value * obs;

            if (config.check("syncObserver", obs, "synchronization signal observer"))
            {
                auto * observer = *reinterpret_cast<TypedStateObserver<double> * const *>(obs->asBlob());
                syncThread->setObserver(observer);
            }
        }
    }
    else
    {
        yCWarning(CBB) << "Synchronization thread not enabled, use --syncPeriod";
    }

    return !syncThread || syncThread->start();
}

// -----------------------------------------------------------------------------

bool CanBusBroker::close()
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
        delete broker;
    }

    brokers.clear();

    for (auto * device : nodeDevices)
    {
        // CAN read threads must not live beyond this point.
        ok &= device->close();
        delete device;
    }

    nodeDevices.clear();

    for (auto * device : busDevices)
    {
        ok &= device->close();
        delete device;
    }

    busDevices.clear();

    return ok;
}

// -----------------------------------------------------------------------------
