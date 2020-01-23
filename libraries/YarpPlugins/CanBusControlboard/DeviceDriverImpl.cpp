// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include <ColorDebug.h>

#include "ICanBusSharer.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::open(yarp::os::Searchable & config)
{
    CD_DEBUG("%s\n", config.toString().c_str());

    if (!config.check("robotConfig") || !config.find("robotConfig").isBlob())
    {
        CD_ERROR("Missing \"robotConfig\" property or not a blob.\n");
        return false;
    }

    const auto * robotConfig = *reinterpret_cast<yarp::os::Property * const *>(config.find("robotConfig").asBlob());

    yarp::os::Bottle * canBuses = config.find("buses").asList();

    if (canBuses == nullptr)
    {
        CD_ERROR("Missing key \"buses\" or not a list.\n");
        return false;
    }

    int fakeNodeCount = 0;

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
                CD_ERROR("Missing CAN bus device group %s.\n", canBus.c_str());
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
            CD_ERROR("canBusDevice instantiation not worked: %s.\n", canBus.c_str());
            return false;
        }

        if (!isFakeBus)
        {
            CanBusBroker * canBusBroker = new CanBusBroker(canBus);
            canBusBrokers.push_back(canBusBroker);

            if (!canBusBroker->configure(canBusOptions))
            {
                CD_ERROR("Unable to configure broker of CAN bus device %s.\n", canBus.c_str());
                return false;
            }

            if (!canBusBroker->registerDevice(canBusDevice))
            {
                CD_ERROR("Unable to register CAN bus device %s.\n", canBus.c_str());
                return false;
            }
        }

        if (!config.check(canBus))
        {
            CD_ERROR("Missing key \"%s\".\n", canBus.c_str());
            return false;
        }

        yarp::os::Value & nodesVal = config.find(canBus);
        yarp::os::Bottle nodes;

        if (!isFakeBus)
        {
            if (nodesVal.asList() == nullptr)
            {
                CD_ERROR("Key \"%s\" must be a list.\n", canBus.c_str());
                return false;
            }

            nodes = yarp::os::Bottle(*nodesVal.asList());
        }
        else
        {
            if (!nodesVal.isInt32())
            {
                CD_ERROR("Key \"%s\" must hold an integer value (number of fake nodes).\n", canBus.c_str());
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
                    CD_ERROR("Missing CAN node device group %s.\n", node.c_str());
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
                CD_ERROR("CAN node device %s configuration failure.\n", node.c_str());
                return false;
            }

            if (!deviceMapper.registerDevice(device))
            {
                CD_ERROR("Unable to register CAN node device %s.\n", node.c_str());
                return false;
            }

            if (!isFakeNode)
            {
                ICanBusSharer * iCanBusSharer;

                if (!device->view(iCanBusSharer))
                {
                    CD_ERROR("Unable to view ICanBusSharer in %s.\n", node.c_str());
                    return false;
                }

                canBusBrokers.back()->getReader()->registerHandle(iCanBusSharer);
                iCanBusSharer->registerSender(canBusBrokers.back()->getWriter()->getDelegate());
            }
            else
            {
                fakeNodeCount++;
            }
        }

        bool enableAcceptanceFilters = canBusOptions.check("enableAcceptanceFilters", yarp::os::Value(false),
                "enable CAN acceptance filters").asBool();

        if (enableAcceptanceFilters && !canBusBrokers.back()->addFilters())
        {
            CD_ERROR("Unable to register CAN acceptance filters in %s.\n", canBus.c_str());
            return false;
        }
    }

    int threadedAxes = deviceMapper.getControlledAxes() - fakeNodeCount;

    // FIXME: temporarily disabled
    if (false /*config.check("threaded", yarp::os::Value(false), "use threads to map joint calls").asBool() && threadedAxes != 0*/)
    {
        // twice as many controlled axes to account for CBW's periodic thread and user RPC requests
        deviceMapper.enableParallelization(threadedAxes * 2);
    }

    for (auto * canBusBroker : canBusBrokers)
    {
        if (!canBusBroker->startThreads())
        {
            CD_ERROR("Unable to start CAN threads in %s.\n", canBusBroker->getName().c_str());
            return false;
        }
    }

    for (const auto & t : deviceMapper.getDevicesWithOffsets())
    {
        auto * iCanBusSharer = std::get<0>(t)->castToType<ICanBusSharer>();

        if (!iCanBusSharer->initialize())
        {
            CD_ERROR("Node device id %d could not initialize CAN comms.\n", iCanBusSharer->getId());
        }
    }

    if (config.check("syncPeriod", "SYNC message period (s)"))
    {
        double syncPeriod = config.find("syncPeriod").asFloat64();

        syncTimer = new yarp::os::Timer(yarp::os::TimerSettings(syncPeriod),
                [this](const yarp::os::YarpTimerEvent & event)
                {
                    for (auto * canBusBroker : canBusBrokers)
                    {
                        auto * reader = canBusBroker->getReader();
                        auto * writer = canBusBroker->getWriter();

                        for (const auto & entry : reader->getHandleMap())
                        {
                            entry.second->synchronize();
                        }

                        writer->getDelegate()->prepareMessage({0x80, 0, nullptr}); // SYNC
                        writer->flush();
                    }

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

    for (const auto & t : deviceMapper.getDevicesWithOffsets())
    {
        auto * iCanBusSharer = std::get<0>(t)->castToType<ICanBusSharer>();

        if (iCanBusSharer && !iCanBusSharer->finalize())
        {
            CD_WARNING("Node device id %d could not finalize CAN comms.\n", iCanBusSharer->getId());
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
