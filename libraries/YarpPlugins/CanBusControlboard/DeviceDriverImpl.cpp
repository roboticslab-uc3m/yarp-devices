// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <map>
#include <string>

#include "ITechnosoftIpos.h"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::open(yarp::os::Searchable & config)
{
    int cuiTimeout  = config.check("waitEncoder", yarp::os::Value(DEFAULT_CUI_TIMEOUT), "CUI timeout (seconds)").asInt32();

    std::string canBusType = config.check("canBusType", yarp::os::Value(DEFAULT_CAN_BUS), "CAN bus device name").asString();
    int canRxBufferSize = config.check("canBusRxBufferSize", yarp::os::Value(DEFAULT_CAN_RX_BUFFER_SIZE), "CAN bus RX buffer size").asInt();
    int canTxBufferSize = config.check("canBusTxBufferSize", yarp::os::Value(DEFAULT_CAN_TX_BUFFER_SIZE), "CAN bus TX buffer size").asInt();
    double canRxPeriodMs = config.check("canRxPeriodMs", yarp::os::Value(DEFAULT_CAN_RX_PERIOD_MS), "CAN bus RX period (milliseconds)").asFloat64();
    double canTxPeriodMs = config.check("canTxPeriodMs", yarp::os::Value(DEFAULT_CAN_TX_PERIOD_MS), "CAN bus TX period (milliseconds)").asFloat64();
    double canSdoTimeoutMs = config.check("canSdoTimeoutMs", yarp::os::Value(DEFAULT_CAN_SDO_TIMEOUT_MS), "CAN bus SDO timeout (milliseconds)").asFloat64();
    double canDriveStateTimeout = config.check("canDriveStateTimeout", yarp::os::Value(DEFAULT_CAN_DRIVE_STATE_TIMEOUT), "CAN drive state timeout (seconds)").asFloat64();

    linInterpPeriodMs = config.check("linInterpPeriodMs", yarp::os::Value(DEFAULT_LIN_INTERP_PERIOD_MS), "linear interpolation mode period (milliseconds)").asInt32();
    linInterpBufferSize = config.check("linInterpBufferSize", yarp::os::Value(DEFAULT_LIN_INTERP_BUFFER_SIZE), "linear interpolation mode buffer size").asInt32();
    linInterpMode = config.check("linInterpMode", yarp::os::Value(DEFAULT_LIN_INTERP_MODE), "linear interpolation mode (pt/pvt)").asString();

    yarp::os::Bottle ids = config.findGroup("ids", "CAN bus IDs").tail();  //-- e.g. 15
    yarp::os::Bottle trs = config.findGroup("trs", "reductions").tail();  //-- e.g. 160
    yarp::os::Bottle ks = config.findGroup("ks", "motor constants").tail();  //-- e.g. 0.0706

    yarp::os::Bottle maxs = config.findGroup("maxs", "maximum joint limits (meters or degrees)").tail();  //-- e.g. 360
    yarp::os::Bottle mins = config.findGroup("mins", "minimum joint limits (meters or degrees)").tail();  //-- e.g. -360
    yarp::os::Bottle maxVels = config.findGroup("maxVels", "maximum joint velocities (meters/second or degrees/second)").tail();  //-- e.g. 1000
    yarp::os::Bottle refAccelerations = config.findGroup("refAccelerations", "ref accelerations (meters/second^2 or degrees/second^2)").tail();  //-- e.g. 0.575437
    yarp::os::Bottle refSpeeds = config.findGroup("refSpeeds", "ref speeds (meters/second or degrees/second)").tail();  //-- e.g. 737.2798
    yarp::os::Bottle encoderPulsess = config.findGroup("encoderPulsess", "encoder pulses (multiple nodes)").tail();  //-- e.g. 4096 (4 * 1024)
    yarp::os::Bottle pulsesPerSamples = config.findGroup("pulsesPerSamples", "encoder pulses per sample (multiple nodes)").tail();  //-- e.g. 1000

    yarp::os::Bottle types = config.findGroup("types", "device name of each node").tail();  //-- e.g. 15

    yarp::os::Property canBusOptions;
    canBusOptions.fromString(config.toString());  // canDevice, canBitrate
    canBusOptions.put("device", canBusType);
    canBusOptions.put("canBlockingMode", false); // enforce non-blocking mode
    canBusOptions.put("canAllowPermissive", false); // always check usage requirements
    canBusOptions.setMonitor(config.getMonitor(), canBusType.c_str());

    if (!canBusDevice.open(canBusOptions))
    {
        CD_ERROR("canBusDevice instantiation not worked.\n");
        return false;
    }

    if (!canBusDevice.view(iCanBus))
    {
        CD_ERROR("Cannot view ICanBus interface in device: %s.\n", canBusType.c_str());
        return false;
    }

    yarp::dev::ICanBufferFactory * iCanBufferFactory;

    if (!canBusDevice.view(iCanBufferFactory))
    {
        CD_ERROR("Cannot view ICanBufferFactory interface in device: %s.\n", canBusType.c_str());
        return false;
    }

    std::map<int, ITechnosoftIpos *> idToTechnosoftIpos;
    iCanBusSharers.resize(ids.size());

    for (int i = 0; i < ids.size(); i++)
    {
        if (types.get(i).asString().empty())
        {
            CD_WARNING("Argument \"types\" empty at %d.\n", i);
        }

        yarp::os::Property options;
        options.put("device", types.get(i));
        options.put("canId", ids.get(i));
        options.put("tr", trs.get(i));
        options.put("min", mins.get(i));
        options.put("max", maxs.get(i));
        options.put("maxVel", maxVels.get(i));
        options.put("k", ks.get(i));
        options.put("refAcceleration", refAccelerations.get(i));
        options.put("refSpeed", refSpeeds.get(i));
        options.put("encoderPulses", encoderPulsess.get(i));
        options.put("pulsesPerSample", pulsesPerSamples.get(i));
        options.put("linInterpPeriodMs", linInterpPeriodMs);
        options.put("linInterpBufferSize", linInterpBufferSize);
        options.put("linInterpMode", linInterpMode);
        options.put("canSdoTimeoutMs", canSdoTimeoutMs);
        options.put("canDriveStateTimeout", canDriveStateTimeout);
        options.put("cuiTimeout", cuiTimeout);
        std::string context = types.get(i).asString() + "_" + std::to_string(ids.get(i).asInt32());
        options.setMonitor(config.getMonitor(),context.c_str());

        yarp::dev::PolyDriver * device = new yarp::dev::PolyDriver(options);

        if (!device->isValid())
        {
            CD_ERROR("CAN node [%d] '%s' instantiation failure.\n", i, types.get(i).asString().c_str());
            return false;
        }

        nodes.push(device, ""); // TODO: device key

        if (!deviceMapper.registerDevice(device))
        {
            CD_ERROR("Unable to register device.\n");
            return false;
        }

        if (!device->view(iCanBusSharers[i]))
        {
            CD_ERROR("Unable to view ICanBusSharer.\n");
            return false;
        }

        iCanBusSharers[i]->registerSender(canWriterThread->getDelegate());

        if (types.get(i).asString() == "TechnosoftIpos")
        {
            motorIds.push_back(i);

            ITechnosoftIpos * iTechnosoftIpos;
            device->view(iTechnosoftIpos);
            idToTechnosoftIpos.insert(std::make_pair(i, iTechnosoftIpos));
        }

        if (!iCanBus->canIdAdd(ids.get(i).asInt32()))
        {
            CD_ERROR("Cannot register acceptance filter for node ID: %d.\n", ids.get(i).asInt32());
            return false;
        }
    }

    const std::string canDevice = canBusOptions.find("canDevice").asString();

    canReaderThread = new CanReaderThread(canDevice, iCanBusSharers);
    canReaderThread->setCanHandles(iCanBus, iCanBufferFactory, canRxBufferSize);
    canReaderThread->setPeriod(canRxPeriodMs);
    canReaderThread->start();

    canWriterThread = new CanWriterThread(canDevice);
    canWriterThread->setCanHandles(iCanBus, iCanBufferFactory, canTxBufferSize);
    canWriterThread->setPeriod(canTxPeriodMs);
    canWriterThread->start();

    for (auto p : iCanBusSharers)
    {
        if (!p->initialize())
        {
            return false;
        }
    }

    posdThread = new PositionDirectThread(linInterpPeriodMs * 0.001);
    posdThread->setNodeHandles(idToTechnosoftIpos);
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

    for (auto p : iCanBusSharers)
    {
        ok &= p->finalize();
    }

    for (int i = 0; i < nodes.size(); i++)
    {
        ok &= nodes[i]->poly->close();
        delete nodes[i]->poly;
    }

    if (canWriterThread && canWriterThread->isRunning())
    {
        ok &= canWriterThread->stop();
    }

    delete canWriterThread;

    if (canReaderThread && canReaderThread->isRunning())
    {
        ok &= canReaderThread->stop();
    }

    delete canReaderThread;

    // Clear CAN acceptance filters ('0' = all IDs that were previously set by canIdAdd).
    if (!iCanBus->canIdDelete(0))
    {
        CD_WARNING("CAN filters may be preserved on the next run.\n");
    }

    ok &= canBusDevice.close();
    return ok;
}

// -----------------------------------------------------------------------------
