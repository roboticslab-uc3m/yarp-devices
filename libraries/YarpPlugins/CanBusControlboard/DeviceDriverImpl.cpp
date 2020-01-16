// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include <yarp/dev/CanBusInterface.h>

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
        busDevices.push(canBusDevice, canBus.c_str());

        if (!canBusDevice->open(canBusOptions))
        {
            CD_ERROR("canBusDevice instantiation not worked.\n");
            return false;
        }

        yarp::dev::ICanBus * iCanBus;

        if (!canBusDevice->view(iCanBus))
        {
            CD_ERROR("Cannot view ICanBus interface in device: %s.\n", canBus.c_str());
            return false;
        }

        yarp::dev::ICanBusErrors * iCanBusErrors;

        if (!canBusDevice->view(iCanBusErrors))
        {
            CD_ERROR("Cannot view ICanBusErrors interface in device: %s.\n", canBus.c_str());
            return false;
        }

        yarp::dev::ICanBufferFactory * iCanBufferFactory;

        if (!canBusDevice->view(iCanBufferFactory))
        {
            CD_ERROR("Cannot view ICanBufferFactory interface in device: %s.\n", canBus.c_str());
            return false;
        }

        if (!isFakeBus)
        {
            int rxBufferSize = canBusOptions.check("rxBufferSize", yarp::os::Value(100), "CAN bus RX buffer size").asInt32();
            int txBufferSize = canBusOptions.check("txBufferSize", yarp::os::Value(100), "CAN bus TX buffer size").asInt32();
            double rxPeriod = canBusOptions.check("rxPeriod", yarp::os::Value(0.0), "CAN bus RX period (seconds)").asFloat64();
            double txPeriod = canBusOptions.check("txPeriod", yarp::os::Value(0.0), "CAN bus TX period (seconds)").asFloat64();

            CanReaderThread * reader = new CanReaderThread(canBus);
            reader->setCanHandles(iCanBus, iCanBusErrors, iCanBufferFactory, rxBufferSize);
            reader->setPeriod(rxPeriod);

            CanWriterThread * writer = new CanWriterThread(canBus);
            writer->setCanHandles(iCanBus, iCanBusErrors, iCanBufferFactory, txBufferSize);
            writer->setPeriod(txPeriod);

            canThreads.push_back({canBus, reader, writer});
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

        bool enableAcceptanceFilters = canBusOptions.check("enableAcceptanceFilters", yarp::os::Value(false),
                "enable CAN acceptance filters").asBool();

        std::vector<unsigned int> filterIds;

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
            nodeDevices.push(device, node.c_str());

            if (!device->open(nodeOptions))
            {
                CD_ERROR("CAN node device %s configuration failure.\n", node.c_str());
                return false;
            }

            if (!deviceMapper.registerDevice(device))
            {
                CD_ERROR("Unable to register device %s.\n", node.c_str());
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

                canThreads.back().reader->registerHandle(iCanBusSharer);
                iCanBusSharer->registerSender(canThreads.back().writer->getDelegate());

                if (enableAcceptanceFilters)
                {
                    auto additionalIds = iCanBusSharer->getAdditionalIds();
                    filterIds.push_back(iCanBusSharer->getId());
                    filterIds.insert(filterIds.end(), additionalIds.begin(), additionalIds.end());
                }
            }
            else
            {
                fakeNodeCount++;
            }
        }

        for (auto id : filterIds)
        {
            if (!iCanBus->canIdAdd(id))
            {
                CD_ERROR("Unable to register acceptance filter id %d in %s.\n", id, canBus.c_str());
                return false;
            }
        }
    }

    int threadedAxes = deviceMapper.getControlledAxes() - fakeNodeCount;

    // FIXME: temporarily disabled
    if (false /*config.check("threaded", yarp::os::Value(false), "use threads to map joint calls").asBool() && threadedAxes != 0*/)
    {
        // twice as many controlled axes to account for CBW's periodic thread and user RPC requests
        deviceMapper.enableParallelization(threadedAxes * 2);
    }

    for (const auto & bundle : canThreads)
    {
        if (!bundle.reader->start())
        {
            CD_ERROR("Unable to start reader thread.\n");
            return false;
        }

        if (!bundle.writer->start())
        {
            CD_ERROR("Unable to start writer thread.\n");
            return false;
        }
    }

    for (int i = 0; i < nodeDevices.size(); i++)
    {
        ICanBusSharer * iCanBusSharer;
        nodeDevices[i]->poly->view(iCanBusSharer);

        if (!iCanBusSharer->initialize())
        {
            CD_ERROR("Node device %s could not initialize CAN comms.\n", nodeDevices[i]->key.c_str());
        }
    }

    if (config.check("syncPeriod", "SYNC message period (s)"))
    {
        double syncPeriod = config.find("syncPeriod").asFloat64();

        syncTimer = new yarp::os::Timer(yarp::os::TimerSettings(syncPeriod),
                [this](const yarp::os::YarpTimerEvent & event)
                {
                    for (const auto & bundle : canThreads)
                    {
                        auto * reader = bundle.reader;
                        auto * writer = bundle.writer;

                        for (const auto & entry : reader->getHandleMap())
                        {
                            entry.second->synchronize();
                        }

                        writer->getDelegate()->prepareMessage({0x80, 0, nullptr}); // SYNC
                        writer->step();
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

    for (int i = 0; i < nodeDevices.size(); i++)
    {
        if (nodeDevices[i]->poly)
        {
            ICanBusSharer * iCanBusSharer;
            nodeDevices[i]->poly->view(iCanBusSharer);

            if (!iCanBusSharer->finalize())
            {
                CD_WARNING("Node device %s could not finalize CAN comms.\n", nodeDevices[i]->key.c_str());
                ok = false;
            }
        }
    }

    for (const auto & bundle : canThreads)
    {
        if (bundle.reader && bundle.reader->isRunning())
        {
            bundle.reader->stop();
        }

        delete bundle.reader;

        if (bundle.writer && bundle.writer->isRunning())
        {
            bundle.writer->stop();
        }

        delete bundle.writer;
    }

    canThreads.clear();

    for (int i = 0; i < nodeDevices.size(); i++)
    {
        if (nodeDevices[i]->poly)
        {
            // CAN read threads must not live beyond this point.
            ok &= nodeDevices[i]->poly->close();
        }

        delete nodeDevices[i]->poly;
        nodeDevices[i]->poly = nullptr;
    }

    for (int i = 0; i < busDevices.size(); i++)
    {
        if (busDevices[i]->poly)
        {
            if (busDevices[i]->poly->getValue("enableAcceptanceFilters").asBool())
            {
                yarp::dev::ICanBus * iCanBus;
                busDevices[i]->poly->view(iCanBus);

                // Clear CAN acceptance filters ('0' = all IDs that were previously set by canIdAdd).
                if (!iCanBus->canIdDelete(0))
                {
                    CD_WARNING("CAN filters on bus %s may be preserved on the next run.\n", busDevices[i]->key.c_str());
                }
            }

            ok &= busDevices[i]->poly->close();
        }

        delete busDevices[i]->poly;
        busDevices[i]->poly = nullptr;
    }

    return ok;
}

// -----------------------------------------------------------------------------
