// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <map>
#include <string>

#include "ITechnosoftIpos.h"

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CanBusControlboard::open(yarp::os::Searchable& config)
{
    std::string mode = config.check("mode",yarp::os::Value("position"),"control mode on startup (position/velocity)").asString();
    int cuiTimeout  = config.check("waitEncoder", yarp::os::Value(DEFAULT_CUI_TIMEOUT), "CUI timeout (seconds)").asInt32();
    bool homing = config.check("home", yarp::os::Value(false), "perform homing maneuver on start").asBool();

    std::string canBusType = config.check("canBusType", yarp::os::Value(DEFAULT_CAN_BUS), "CAN bus device name").asString();
    int canRxBufferSize = config.check("canBusRxBufferSize", yarp::os::Value(DEFAULT_CAN_RX_BUFFER_SIZE), "CAN bus RX buffer size").asInt();
    int canTxBufferSize = config.check("canBusTxBufferSize", yarp::os::Value(DEFAULT_CAN_TX_BUFFER_SIZE), "CAN bus TX buffer size").asInt();
    double canRxPeriodMs = config.check("canRxPeriodMs", yarp::os::Value(DEFAULT_CAN_RX_PERIOD_MS), "CAN bus RX period (milliseconds)").asFloat64();
    double canTxPeriodMs = config.check("canTxPeriodMs", yarp::os::Value(DEFAULT_CAN_TX_PERIOD_MS), "CAN bus TX period (milliseconds)").asFloat64();
    double canSdoTimeoutMs = config.check("canSdoTimeoutMs", yarp::os::Value(DEFAULT_CAN_SDO_TIMEOUT_MS), "CAN bus SDO timeout (milliseconds)").asFloat64();

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

    //-- Initialize the CAN device.
    yarp::os::Property canBusOptions;
    canBusOptions.fromString(config.toString());  // canDevice, canBitrate
    canBusOptions.put("device", canBusType);
    canBusOptions.put("canBlockingMode", false); // enforce non-blocking mode
    canBusOptions.put("canAllowPermissive", false); // always check usage requirements
    canBusOptions.setMonitor(config.getMonitor(), canBusType.c_str());
    canBusDevice.open(canBusOptions);
    if( ! canBusDevice.isValid() )
    {
        CD_ERROR("canBusDevice instantiation not worked.\n");
        return false;
    }

    if( !canBusDevice.view(iCanBus) )
    {
        CD_ERROR("Cannot view ICanBus interface in device: %s.\n", canBusType.c_str());
        return false;
    }

    yarp::dev::ICanBufferFactory * iCanBufferFactory;

    if( !canBusDevice.view(iCanBufferFactory) )
    {
        CD_ERROR("Cannot view ICanBufferFactory interface in device: %s.\n", canBusType.c_str());
        return false;
    }

    std::string canDevice = canBusOptions.find("canDevice").asString();

    //-- Start the reading thread (required for checkMotionDoneRaw).
    canReaderThread = new CanReaderThread(canDevice, idxFromCanId, iCanBusSharer);
    canReaderThread->setCanHandles(iCanBus, iCanBufferFactory, canRxBufferSize);
    canReaderThread->setPeriod(canRxPeriodMs);
    canReaderThread->start();

    canWriterThread = new CanWriterThread(canDevice);
    canWriterThread->setCanHandles(iCanBus, iCanBufferFactory, canTxBufferSize);
    canWriterThread->setPeriod(canTxPeriodMs);
    canWriterThread->start();

    posdThread = new PositionDirectThread(linInterpPeriodMs * 0.001);

    //-- Populate the CAN nodes vector.
    nodes.resize( ids.size() );
    iControlLimitsRaw.resize( nodes.size() );
    iControlModeRaw.resize( nodes.size() );
    iCurrentControlRaw.resize( nodes.size() );
    iEncodersTimedRaw.resize( nodes.size() );
    iInteractionModeRaw.resize( nodes.size() );
    iPositionControlRaw.resize( nodes.size() );
    iPositionDirectRaw.resize( nodes.size() );
    iRemoteVariablesRaw.resize( nodes.size() );
    iTorqueControlRaw.resize( nodes.size() );
    iVelocityControlRaw.resize( nodes.size() );
    iCanBusSharer.resize( nodes.size() );

    std::map<int, ITechnosoftIpos *> idToTechnosoftIpos;
    std::map<int, int> technosoftToNodeId;

    for(int i=0; i<nodes.size(); i++)
    {
        if(types.get(i).asString() == "")
            CD_WARNING("Argument \"types\" empty at %d.\n",i);

        //-- Create CAN node objects with a pointer to the CAN device, its id and tr (these are locally stored parameters).
        yarp::os::Property options;
        options.put("device", types.get(i));  //-- "TechnosoftIpos", "LacqueyFetch", "CuiAbsolute"
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
        options.put("cuiTimeout", cuiTimeout);
        std::string context = types.get(i).asString() + "_" + std::to_string(ids.get(i).asInt32());
        options.setMonitor(config.getMonitor(),context.c_str());

        // -- Configuramos todos los dispositivos (TechnosoftIpos, LacqueyFetch, CuiAbsolute)
        yarp::dev::PolyDriver* device = new yarp::dev::PolyDriver(options);
        if( ! device->isValid() )
        {
            CD_ERROR("CAN node [%d] '%s' instantiation not worked.\n",i,types.get(i).asString().c_str());
            return false;
        }

        //-- Fill a map entry ( drivers.size() if before push_back, otherwise do drivers.size()-1).
        //-- Just "i" if resize already performed.
        idxFromCanId[ ids.get(i).asInt32() ] = i;

        //-- Push the motor driver and other devices (CuiAbsolute) on to the vectors.
        nodes[i] = device;

        //-- View interfaces
        if( !device->view( iControlLimitsRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iControlLimits2Raw interface\n");
            return false;
        }

        if( !device->view( iControlModeRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iControlMode2Raw interface\n");
            return false;
        }

        if( !device->view( iCurrentControlRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iCurrentControlRaw interface\n");
            return false;
        }

        if( !device->view( iEncodersTimedRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iEncodersTimedRaw interface\n");
            return false;
        }

        if( !device->view( iInteractionModeRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iInteractionModeRaw interface\n");
            return false;
        }

        if( !device->view( iPositionControlRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iPositionControlRaw interface\n");
            return false;
        }

        if( !device->view( iPositionDirectRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iPositionDirectRaw interface\n");
            return false;
        }

        if( !device->view( iRemoteVariablesRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iRemoteVariablesRaw interface\n");
            return false;
        }

        if( !device->view( iTorqueControlRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iTorqueControlRaw interface\n");
            return false;
        }

        if( !device->view( iVelocityControlRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iVelocityControl2Raw interface\n");
            return false;
        }

        // -- si el device es un Cui, este podrá "ver" las funciones programadas en iCanBusSharer (funciones que hemos añadido al encoder).
        // -- estas funciones se encuentran implementadas en el cpp correspondiente "ICanBusSharerImpl.cpp", por lo tanto le da la funcionalidad que deseamos
        if( !device->view( iCanBusSharer[i] ))
        {
            CD_ERROR("[error] Problems acquiring iCanBusSharer interface\n");
            return false;
        }

        iCanBusSharer[i]->registerSender(canWriterThread->getDelegate());

        //-- DRIVERS
        if(types.get(i).asString() == "TechnosoftIpos")
        {
            motorIds.push_back(i);

            ITechnosoftIpos * iTechnosoftIpos;
            device->view(iTechnosoftIpos);
            idToTechnosoftIpos.insert(std::make_pair(i, iTechnosoftIpos));

            technosoftToNodeId.insert(std::make_pair(i, ids.get(i).asInt32()));
        }

        //-- Associate absolute encoders to motor drivers
        if( types.get(i).asString() == "CuiAbsolute" )
        {
            int driverCanId = ids.get(i).asInt32() - 100;  //-- \todo{Document the dangers: ID must be > 100, driver must be instanced.}
            iCanBusSharer[ idxFromCanId[driverCanId] ]->setIEncodersTimedRawExternal( iEncodersTimedRaw[i] );
        }

        //-- Enable acceptance filters for each node ID
        if( !iCanBus->canIdAdd(ids.get(i).asInt32()) )
        {
            CD_ERROR("Cannot register acceptance filter for node ID: %d.\n", ids.get(i).asInt32());
            return false;
        }

    } // -- for(int i=0; i<nodes.size(); i++)

    //-- Set all motor drivers to mode.

    int controlModeVocab = 0;

    if( mode=="position" )
        controlModeVocab = VOCAB_CM_POSITION;
    else if( mode=="velocity" )
        controlModeVocab = VOCAB_CM_VELOCITY;
    else if( mode=="torque" )
        controlModeVocab = VOCAB_CM_TORQUE;
    else
    {
        CD_ERROR("Not prepared for initializing in mode %s.\n",mode.c_str());
        return false;
    }

    for(int i=0; i<nodes.size(); i++)
    {
        if( ! this->setControlMode(i, controlModeVocab) )
            return false;
    }

    //-- Initialize the drivers.
    for (auto node : iCanBusSharer)
    {
        if (!node->initialize() || !node->start() || !node->readyToSwitchOn() || !node->switchOn() || !node->enable())
        {
            return false;
        }
    }

    //-- Homing
    if (homing)
    {
        CD_DEBUG("Moving motors to zero.\n");

        for (auto entry : technosoftToNodeId)
        {
            int i = entry.first;
            yarp::os::Time::delay(0.5);

            double val;
            double time;
            iEncodersTimedRaw[i]->getEncoderTimedRaw(0, &val, &time);

            CD_DEBUG("Value of relative encoder -> %f\n", val);

            if (val > 0.087873 || val< -0.087873)
            {
                CD_DEBUG("Moving (ID:%d) to zero...\n", entry.second);

                if (!iPositionControlRaw[i]->positionMoveRaw(0, 0))
                {
                    return false;
                }
            }
            else
            {
                CD_DEBUG("It's already in zero position.\n");
            }
        }

        // -- Testing
        for (auto entry : technosoftToNodeId)
        {
            int i = entry.first;
            bool motionDone = false;
            yarp::os::Time::delay(0.2);

            CD_DEBUG("Testing (ID:%s) position...\n", entry.second);

            if (!iPositionControlRaw[i]->checkMotionDoneRaw(0, &motionDone))
            {
                return false;
            }

            if (!motionDone)
            {
                CD_WARNING("Test motion fail (ID:%d)\n", entry.second);
            }
        }

        CD_DEBUG("Moved motors to zero.\n");
        yarp::os::Time::delay(1);
    }

    if( config.check("reset", "reset encoders to zero") )
    {
        CD_DEBUG("Forcing encoders to zero.\n");
        if ( ! this->resetEncoders() )
            return false;
    }

    posdThread->setNodeHandles(idToTechnosoftIpos);
    posdThread->start();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::close()
{
    bool ok = true;

    if (posdThread && posdThread->isRunning())
    {
        posdThread->stop();
    }

    delete posdThread;

    //-- Delete the driver objects.
    for (int i = 0; i < nodes.size(); i++)
    {
        ok &= nodes[i]->close();
        delete nodes[i];
        nodes[i] = 0;
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

    //-- Clear CAN acceptance filters ('0' = all IDs that were previously set by canIdAdd).
    if (!iCanBus->canIdDelete(0))
    {
        CD_WARNING("CAN filters may be preserved on the next run.\n");
    }

    ok &= canBusDevice.close();
    return ok;
}

// -----------------------------------------------------------------------------
