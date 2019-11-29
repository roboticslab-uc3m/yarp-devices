// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Property.h>
#include <yarp/dev/CanBusInterface.h>

#include <ColorDebug.h>

#include "ICanBusSharer.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::open(yarp::os::Searchable & config)
{
    CD_DEBUG("%s\n", config.toString().c_str());

    int parallelCanThreadLimit = config.check("parallelCanThreadLimit", yarp::os::Value(0), "parallel CAN TX thread limit").asInt32();

    if (parallelCanThreadLimit > 0)
    {
        deviceMapper.enableParallelization(parallelCanThreadLimit);
    }

    yarp::os::Bottle * canBuses = config.find("buses").asList();

    if (canBuses == nullptr)
    {
        CD_ERROR("Missing key \"buses\" or not a list.\n");
        return false;
    }

    canThreads.resize(canBuses->size());

    for (int i = 0; i < canBuses->size(); i++)
    {
        std::string canBus = canBuses->get(i).asString();

        yarp::os::Bottle & canBusGroup = config.findGroup(canBus);

        if (canBusGroup.isNull())
        {
            CD_ERROR("Missing CAN bus device group %s.\n", canBus.c_str());
            return false;
        }

        yarp::os::Property canBusOptions;
        canBusOptions.fromString(canBusGroup.toString(), false);
        canBusOptions.fromString(config.toString(), false);
        canBusOptions.put("canBlockingMode", false); // enforce non-blocking mode
        canBusOptions.put("canAllowPermissive", false); // always check usage requirements
        canBusOptions.setMonitor(config.getMonitor(), canBus.c_str());

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

        yarp::dev::ICanBufferFactory * iCanBufferFactory;

        if (!canBusDevice->view(iCanBufferFactory))
        {
            CD_ERROR("Cannot view ICanBufferFactory interface in device: %s.\n", canBus.c_str());
            return false;
        }

        int canRxBufferSize = config.check("canBusRxBufferSize", yarp::os::Value(DEFAULT_CAN_RX_BUFFER_SIZE), "CAN bus RX buffer size").asInt();
        int canTxBufferSize = config.check("canBusTxBufferSize", yarp::os::Value(DEFAULT_CAN_TX_BUFFER_SIZE), "CAN bus TX buffer size").asInt();
        double canRxPeriodMs = config.check("canRxPeriodMs", yarp::os::Value(DEFAULT_CAN_RX_PERIOD_MS), "CAN bus RX period (milliseconds)").asFloat64();
        double canTxPeriodMs = config.check("canTxPeriodMs", yarp::os::Value(DEFAULT_CAN_TX_PERIOD_MS), "CAN bus TX period (milliseconds)").asFloat64();

        CanReaderThread * reader = new CanReaderThread(canBus);
        reader->setCanHandles(iCanBus, iCanBufferFactory, canRxBufferSize);
        reader->setPeriod(canRxPeriodMs);

        CanWriterThread * writer = new CanWriterThread(canBus);
        writer->setCanHandles(iCanBus, iCanBufferFactory, canTxBufferSize);
        writer->setPeriod(canTxPeriodMs);

        canThreads[i].reader = reader;
        canThreads[i].writer = writer;

        yarp::os::Bottle * nodes = config.find(canBus).asList();

        if (nodes == nullptr)
        {
            CD_ERROR("Missing key \"%s\" or not a list.\n", canBus.c_str());
            return false;
        }

        for (int i = 0; i < nodes->size(); i++)
        {
            std::string node = nodes->get(i).asString();
            yarp::os::Bottle & nodeGroup = config.findGroup(node);

            if (nodeGroup.isNull())
            {
                CD_ERROR("Missing CAN node device group %s.\n", node.c_str());
                return false;
            }

            yarp::os::Property nodeOptions;
            nodeOptions.fromString(nodeGroup.toString(), false);
            nodeOptions.fromString(config.toString(), false);
            nodeOptions.setMonitor(config.getMonitor(), node.c_str());

            yarp::dev::PolyDriver * device = new yarp::dev::PolyDriver;
            nodeDevices.push(device, node.c_str());

            if (!device->open(nodeOptions))
            {
                CD_ERROR("CAN node device %s configuration failure.\n", node.c_str());
                return false;
            }

            if (!deviceMapper.registerDevice(device))
            {
                CD_ERROR("Unable to register device.\n");
                return false;
            }

            ICanBusSharer * iCanBusSharer;

            if (!device->view(iCanBusSharer))
            {
                CD_ERROR("Unable to view ICanBusSharer.\n");
                return false;
            }

            reader->registerHandle(iCanBusSharer);
            iCanBusSharer->registerSender(writer->getDelegate());

            if (!iCanBus->canIdAdd(iCanBusSharer->getId()))
            {
                CD_ERROR("Cannot register acceptance filter for node ID: %d.\n", iCanBusSharer->getId());
                return false;
            }
        }
    }

    linInterpPeriodMs = config.check("linInterpPeriodMs", yarp::os::Value(DEFAULT_LIN_INTERP_PERIOD_MS), "linear interpolation mode period (milliseconds)").asInt32();
    linInterpBufferSize = config.check("linInterpBufferSize", yarp::os::Value(DEFAULT_LIN_INTERP_BUFFER_SIZE), "linear interpolation mode buffer size").asInt32();
    linInterpMode = config.check("linInterpMode", yarp::os::Value(DEFAULT_LIN_INTERP_MODE), "linear interpolation mode (pt/pvt)").asString();

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

    for (int i = 0; i > nodeDevices.size(); i++)
    {
        ICanBusSharer * iCanBusSharer;
        nodeDevices[i]->poly->view(iCanBusSharer);

        if (!iCanBusSharer->initialize())
        {
            return false;
        }
    }

    posdThread = new PositionDirectThread(deviceMapper, linInterpPeriodMs * 0.001);
    posdThread->start();

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::close()
{
    bool ok = true;

    if (posdThread && posdThread->isRunning())
    {
        posdThread->stop();
    }

    delete posdThread;

    for (int i = 0; i < nodeDevices.size(); i++)
    {
        ICanBusSharer * iCanBusSharer;
        nodeDevices[i]->poly->view(iCanBusSharer);
        ok &= iCanBusSharer->finalize();

        ok &= nodeDevices[i]->poly->close();
        delete nodeDevices[i]->poly;
    }

    for (const auto & bundle : canThreads)
    {
        if (bundle.reader && bundle.reader->isRunning())
        {
            ok &= bundle.reader->stop();
        }

        delete bundle.reader;

        if (bundle.writer && bundle.writer->isRunning())
        {
            ok &= bundle.writer->stop();
        }

        delete bundle.writer;
    }

    for (int i = 0; i < busDevices.size(); i++)
    {
        yarp::dev::ICanBus * iCanBus;
        busDevices[i]->poly->view(iCanBus);

        // Clear CAN acceptance filters ('0' = all IDs that were previously set by canIdAdd).
        if (!iCanBus->canIdDelete(0))
        {
            CD_WARNING("CAN filters may be preserved on the next run.\n");
        }

        ok &= busDevices[i]->poly->close();
        delete busDevices[i]->poly;
    }

    return ok;
}

// -----------------------------------------------------------------------------
