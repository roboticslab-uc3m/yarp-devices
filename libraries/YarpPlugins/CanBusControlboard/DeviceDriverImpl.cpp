// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include "ICanBusSharer.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::open(yarp::os::Searchable & config)
{
    yDebug() << "CanBusControlboard config:" << config.toString();

    if (!config.check("robotConfig") || !config.find("robotConfig").isBlob())
    {
        yError() << "Missing \"robotConfig\" property or not a blob";
        return false;
    }

    const auto * robotConfig = *reinterpret_cast<yarp::os::Property * const *>(config.find("robotConfig").asBlob());

    yarp::os::Bottle * canBuses = config.find("buses").asList();

    if (canBuses == nullptr)
    {
        yError() << "Missing key \"buses\" or not a list";
        return false;
    }

    for (int i = 0; i < canBuses->size(); i++)
    {
        std::string canBus = canBuses->get(i).asString();
        bool isFakeBus = canBus.find("fake") != std::string::npos;

        yarp::os::Property canBusOptions;
        canBusOptions.setMonitor(config.getMonitor(), canBus.c_str());

        if (!isFakeBus)
        {
            yarp::os::Bottle & canBusGroup = robotConfig->findGroup(canBus);

            if (canBusGroup.isNull())
            {
                yError() << "Missing CAN bus device group" << canBus;
                return false;
            }

            canBusOptions.fromString(canBusGroup.toString());
            canBusOptions.put("robotConfig", config.find("robotConfig"));
            canBusOptions.put("blockingMode", false); // enforce non-blocking mode
            canBusOptions.put("allowPermissive", false); // always check usage requirements
        }
        else
        {
            canBusOptions.put("device", "CanBusFake");
        }

        yarp::dev::PolyDriver * canBusDevice = new yarp::dev::PolyDriver;
        busDevices.push_back(canBusDevice);

        if (!canBusDevice->open(canBusOptions))
        {
            yError() << "canBusDevice instantiation failed:" << canBus;
            return false;
        }

        if (!isFakeBus)
        {
            CanBusBroker * canBusBroker = new CanBusBroker(canBus);
            canBusBrokers.push_back(canBusBroker);

            if (!canBusBroker->configure(canBusOptions))
            {
                yError() << "Unable to configure broker of CAN bus device" << canBus;
                return false;
            }

            if (!canBusBroker->registerDevice(canBusDevice))
            {
                yError() << "Unable to register CAN bus device" << canBus;
                return false;
            }
        }

        if (!config.check(canBus))
        {
            yError() << "Missing key" << canBus;
            return false;
        }

        yarp::os::Value & nodesVal = config.find(canBus);
        yarp::os::Bottle nodes;

        if (!isFakeBus)
        {
            if (nodesVal.asList() == nullptr)
            {
                yError() << "Key" << canBus << "must be a list";
                return false;
            }

            nodes = yarp::os::Bottle(*nodesVal.asList());
        }
        else
        {
            if (!nodesVal.isInt32())
            {
                yError() << "Key" << canBus << "must hold an integer value (number of fake nodes)";
                return false;
            }

            for (int i = 0; i < nodesVal.asInt32(); i++)
            {
                nodes.addString("fake-" + std::to_string(i + 1));
            }
        }

        for (int i = 0; i < nodes.size(); i++)
        {
            std::string node = nodes.get(i).asString();
            bool isFakeNode = node.find("fake") != std::string::npos;

            yarp::os::Property nodeOptions;
            nodeOptions.setMonitor(config.getMonitor(), node.c_str());

            if (!isFakeNode)
            {
                yarp::os::Bottle & nodeGroup = robotConfig->findGroup(node);

                if (nodeGroup.isNull())
                {
                    yError() << "Missing CAN node device group" << node;
                    return false;
                }

                nodeOptions.fromString(nodeGroup.toString());
                nodeOptions.put("robotConfig", config.find("robotConfig"));
                nodeOptions.put("syncPeriod", config.find("syncPeriod"));
            }
            else
            {
                nodeOptions.put("device", "FakeJoint");
            }

            yarp::dev::PolyDriver * device = new yarp::dev::PolyDriver;
            nodeDevices.push_back(device);

            if (!device->open(nodeOptions))
            {
                yError() << "CAN node device" << node << "configuration failure";
                return false;
            }

            if (!deviceMapper.registerDevice(device))
            {
                yError() << "Unable to register CAN node device" << node;
                return false;
            }

            if (!isFakeNode)
            {
                ICanBusSharer * iCanBusSharer;

                if (!device->view(iCanBusSharer))
                {
                    yError() << "Unable to view ICanBusSharer in" << node;
                    return false;
                }

                canBusBrokers.back()->getReader()->registerHandle(iCanBusSharer);
                iCanBusSharer->registerSender(canBusBrokers.back()->getWriter()->getDelegate());
            }
        }

        bool enableAcceptanceFilters = canBusOptions.check("enableAcceptanceFilters", yarp::os::Value(false),
                "enable CAN acceptance filters").asBool();

        if (enableAcceptanceFilters && !canBusBrokers.back()->addFilters())
        {
            yError() << "Unable to register CAN acceptance filters in" << canBus;
            return false;
        }
    }

    for (auto * canBusBroker : canBusBrokers)
    {
        if (!canBusBroker->startThreads())
        {
            yError() << "Unable to start CAN threads in" << canBusBroker->getName();
            return false;
        }
    }

    for (const auto & t : deviceMapper.getDevicesWithOffsets())
    {
        auto * iCanBusSharer = std::get<0>(t)->castToType<ICanBusSharer>();

        if (!iCanBusSharer->initialize())
        {
            yError() << "Node device id" << iCanBusSharer->getId() << "could not initialize CAN comms";
        }
    }

    if (config.check("syncPeriod", "SYNC message period (s)"))
    {
        double syncPeriod = config.find("syncPeriod").asFloat64();
        bool threadedSync = config.check("threadedSync", yarp::os::Value(false), "parallelize SYNC requests").asBool();

        if (busDevices.size() > 1 && threadedSync)
        {
            taskFactory = new ParallelTaskFactory(busDevices.size());
        }
        else
        {
            taskFactory = new SequentialTaskFactory;
        }

        syncTimer = new yarp::os::Timer(yarp::os::TimerSettings(syncPeriod),
            [this](const yarp::os::YarpTimerEvent & event)
            {
                auto task = taskFactory->createTask();

                for (auto * canBusBroker : canBusBrokers)
                {
                    task->add([canBusBroker]
                        {
                            auto * reader = canBusBroker->getReader();
                            auto * writer = canBusBroker->getWriter();

                            for (auto * handle : reader->getHandles())
                            {
                                handle->synchronize();
                            }

                            writer->getDelegate()->prepareMessage({0x80, 0, nullptr}); // SYNC
                            writer->flush();
                            return true;
                        });
                }

                task->dispatch();
                return true;
            },
            true);
    }

    return !syncTimer || syncTimer->start();
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::close()
{
    bool ok = true;

    if (syncTimer && syncTimer->isRunning())
    {
        syncTimer->stop();
    }

    delete syncTimer;
    syncTimer = nullptr;

    delete taskFactory;
    taskFactory = nullptr;

    for (const auto & t : deviceMapper.getDevicesWithOffsets())
    {
        auto * iCanBusSharer = std::get<0>(t)->castToType<ICanBusSharer>();

        if (iCanBusSharer && !iCanBusSharer->finalize())
        {
            yWarning() << "Node device id" << iCanBusSharer->getId() << "could not finalize CAN comms";
            ok = false;
        }
    }

    deviceMapper.clear();

    for (auto * canBusBroker : canBusBrokers)
    {
        ok &= canBusBroker->stopThreads();
        ok &= canBusBroker->clearFilters();
        delete canBusBroker;
    }

    canBusBrokers.clear();

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
