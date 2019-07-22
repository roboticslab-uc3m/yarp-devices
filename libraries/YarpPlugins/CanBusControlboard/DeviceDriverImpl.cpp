// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <map>

#include "ITechnosoftIpos.h"

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CanBusControlboard::open(yarp::os::Searchable& config)
{
    std::string mode = config.check("mode",yarp::os::Value("position"),"control mode on startup (position/velocity)").asString();
    int timeCuiWait  = config.check("waitEncoder", yarp::os::Value(DEFAULT_TIME_TO_WAIT_CUI), "CUI timeout (seconds)").asInt32();
    std::string canBusType = config.check("canBusType", yarp::os::Value(DEFAULT_CAN_BUS), "CAN bus device name").asString();

    yarp::os::Bottle ids = config.findGroup("ids", "CAN bus IDs").tail();  //-- e.g. 15
    yarp::os::Bottle trs = config.findGroup("trs", "reductions").tail();  //-- e.g. 160
    yarp::os::Bottle ks = config.findGroup("ks", "motor constants").tail();  //-- e.g. 0.0706

    yarp::os::Bottle maxs = config.findGroup("maxs", "maximum joint limits (meters or degrees)").tail();  //-- e.g. 360
    yarp::os::Bottle mins = config.findGroup("mins", "minimum joint limits (meters or degrees)").tail();  //-- e.g. -360
    yarp::os::Bottle maxVels = config.findGroup("maxVels", "maximum joint velocities (meters/second or degrees/second)").tail();  //-- e.g. 1000
    yarp::os::Bottle refAccelerations = config.findGroup("refAccelerations", "ref accelerations (meters/second^2 or degrees/second^2)").tail();  //-- e.g. 0.575437
    yarp::os::Bottle refSpeeds = config.findGroup("refSpeeds", "ref speeds (meters/second or degrees/second)").tail();  //-- e.g. 737.2798
    yarp::os::Bottle encoderPulsess = config.findGroup("encoderPulsess", "encoder pulses (multiple nodes)").tail();  //-- e.g. 4096 (4 * 1024)

    yarp::os::Bottle types = config.findGroup("types", "device name of each node").tail();  //-- e.g. 15

    //-- Initialize the CAN device.
    yarp::os::Property canBusOptions;
    canBusOptions.fromString(config.toString());  // canDevice, canBitrate
    canBusOptions.put("device", canBusType);
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

    if( !canBusDevice.view(iCanBufferFactory) )
    {
        CD_ERROR("Cannot view ICanBufferFactory interface in device: %s.\n", canBusType.c_str());
        return false;
    }

    canInputBuffer = iCanBufferFactory->createBuffer(1);

    //-- Start the reading thread (required for checkMotionDoneRaw).
    this->Thread::start();

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
        //std::stringstream ss; // Remember to #include <sstream>
        //ss << types.get(i).asString() << "_" << ids.get(i).asInt32();
        //options.setMonitor(config.getMonitor(),ss.str().c_str());

        yarp::os::Value v(&iCanBufferFactory, sizeof(iCanBufferFactory));
        options.put("canBufferFactory", v);

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

        //-- Pass CAN bus pointer to CAN node
        iCanBusSharer[i]->setCanBusPtr( iCanBus );

        //-- DRIVERS
        if(types.get(i).asString() == "TechnosoftIpos")
        {
            motorIds.push_back(i);

            ITechnosoftIpos * iTechnosoftIpos;
            device->view(iTechnosoftIpos);
            idToTechnosoftIpos.insert(std::make_pair(i, iTechnosoftIpos));

            //-- Set initial parameters on physical motor drivers.

            if ( ! iPositionControlRaw[i]->setRefAccelerationRaw( 0, refAccelerations.get(i).asFloat64() ) )
                return false;

            if ( ! iPositionControlRaw[i]->setRefSpeedRaw( 0, refSpeeds.get(i).asFloat64() ) )
                return false;

            if ( ! iControlLimitsRaw[i]->setLimitsRaw( 0, mins.get(i).asFloat64(), maxs.get(i).asFloat64() ) )
                return false;
        }

        //-- Associate absolute encoders to motor drivers
        if( types.get(i).asString() == "CuiAbsolute" )
        {
            int driverCanId = ids.get(i).asInt32() - 100;  //-- \todo{Document the dangers: ID must be > 100, driver must be instanced.}

            CD_INFO("Sending \"Start Continuous Publishing\" message to Cui Absolute (PIC ID: %d)\n", ids.get(i).asInt32());

            // Configuring Cui Absolute
            ICuiAbsolute* cuiAbsolute;
            if( ! device->view( cuiAbsolute ) )
            {
                CD_ERROR("Could not view.\n");
                return false;
            }

            if ( ! cuiAbsolute->startContinuousPublishing(0) ) // startContinuousPublishing(delay)
                return false;

            yarp::os::Time::delay(0.2);

            if ( timeCuiWait > 0 && ( ! cuiAbsolute->HasFirstReached() ) ) // using --externalEncoderWait && doesn't respond
            {
                bool timePassed = false;
                double timeStamp = 0.0;

                timeStamp = yarp::os::Time::now();

                // This part of the code checks if encoders
                while ( !timePassed && ( ! cuiAbsolute->HasFirstReached() ) )
                {
                    // -- if it exceeds the timeCuiWait...
                    if(int(yarp::os::Time::now()-timeStamp)>=timeCuiWait)
                    {
                        CD_ERROR("Time out passed and CuiAbsolute ID (%d) doesn't respond\n", ids.get(i).asInt32() );
                        yarp::os::Time::delay(2);
                        CD_WARNING("Initializing with normal relative encoder configuration\n");
                        yarp::os::Time::delay(2);
                        timePassed = true;
                    }
                }
            }
            else    // not used --externalEncoderWait (DEFAULT)
            {
                for ( int n=1; n<=5 && ( ! cuiAbsolute->HasFirstReached() ); n++ ) // doesn't respond && trying (5 trials)
                {
                    CD_WARNING("(%d) Resending start continuous publishing message \n", n);
                    if ( ! cuiAbsolute->startContinuousPublishing(0))
                        return false;

                    yarp::os::Time::delay(0.2);
                }

                if( cuiAbsolute->HasFirstReached() ) // it responds! :)
                {
                    CD_DEBUG("---> First CUI message has been reached \n");
                    double value;
                    while( ! iEncodersTimedRaw[i]->getEncoderRaw(0,&value) ){
                        CD_ERROR("Wrong value of Cui \n");
                    }
                    printf("Absolute encoder value -----> %f\n", value);
                    //getchar(); // -- if you want to pause and return pressing any key
                    yarp::os::Time::delay(0.2);
                    iCanBusSharer[ idxFromCanId[driverCanId] ]->setIEncodersTimedRawExternal( iEncodersTimedRaw[i] );
                }
                else                               // doesn't respond :(
                {
                    CD_ERROR("Cui Absolute (PIC ID: %d) doesn't respond. Try using --externalEncoderWait [seconds] parameter with timeout higher than 0 \n", ids.get(i).asInt32());
                    return false;
                }
            }
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

    //-- Check the status of each driver.
    std::vector<int> tmp( nodes.size() ); // -- creating a "tmp"vector with "nodes" vector size
    this->getControlModes( tmp.data() );

    //-- Initialize the drivers: start (0.1) ready (0.1) on (2) enable. Wait between each step.
    for(int i=0; i<nodes.size(); i++)
    {
        if( ! iCanBusSharer[i]->initialize() )
            return false;
    }
    yarp::os::Time::delay(0.1);
    for(int i=0; i<nodes.size(); i++)
    {
        if( ! iCanBusSharer[i]->start() )
            return false;
    }
    yarp::os::Time::delay(0.1);
    for(int i=0; i<nodes.size(); i++)
    {
        if( ! iCanBusSharer[i]->readyToSwitchOn() )
            return false;
    }
    yarp::os::Time::delay(0.1);
    for(int i=0; i<nodes.size(); i++)
    {
        if( ! iCanBusSharer[i]->switchOn() )
            return false;
    }
    yarp::os::Time::delay(2);
    for(int i=0; i<nodes.size(); i++)
    {
        if( ! iCanBusSharer[i]->enable() )
            return false;
    }
    yarp::os::Time::delay(2);

    //-- Homing
    if( config.check("home", "perform homing maneuver on start") )
    {
        CD_DEBUG("Moving motors to zero.\n");
        for(int i=0; i<nodes.size(); i++)
        {
            if((nodes[i]->getValue("device")).asString() == "TechnosoftIpos"){
                double val;
                double time;
                yarp::os::Time::delay(0.5);
                iEncodersTimedRaw[i]->getEncoderTimedRaw(0,&val,&time); // -- getEncoderRaw(0,&value);
                CD_DEBUG("Value of relative encoder ->%f\n", val);
                if ( val>0.087873 || val< -0.087873 ){
                    CD_DEBUG("Moving (ID:%s) to zero...\n",nodes[i]->getValue("canId").toString().c_str());
                    if ( ! iPositionControlRaw[i]->positionMoveRaw(0,0) )
                        return false;
                }
                else
                    CD_DEBUG("It's already in zero position\n");
            }
        }
        // -- Testing

        for(int i=0; i<nodes.size(); i++)
        {
            if((nodes[i]->getValue("device")).asString() == "TechnosoftIpos"){
                bool motionDone = false;
                yarp::os::Time::delay(0.2);  //-- [s]
                CD_DEBUG("Testing (ID:%s) position... \n",nodes[i]->getValue("canId").toString().c_str());
                if( ! iPositionControlRaw[i]->checkMotionDoneRaw(0,&motionDone) )
                    return false;
                if(!motionDone)
                    CD_WARNING("Test motion fail (ID:%s) \n", nodes[i]->getValue("canId").toString().c_str());
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

    posdThread = new PositionDirectThread(0.05);
    posdThread->setNodeHandles(idToTechnosoftIpos);
    posdThread->start();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::close()
{
    const double timeOut = 1; // timeout (1 secod)

    //-- Stop the read thread.
    this->Thread::stop();

    if (posdThread && posdThread->isRunning())
    {
        posdThread->stop();
    }

    delete posdThread;

    const yarp::dev::CanMessage &msg = canInputBuffer[0];

    //-- Disable and shutdown the physical drivers (and Cui Encoders).
    bool ok = true;
    for(int i=0; i<nodes.size(); i++)
    {
        // -- Sending a stop message to PICs of Cui Encoders
        yarp::os::Value value;
        value = nodes[i]->getValue("device");

        // Drivers:
        if(value.asString() == "TechnosoftIpos")
        {
            CD_INFO("Stopping Driver (ID: %s)\n", nodes[i]->getValue("canId").toString().c_str());
            ok &= iCanBusSharer[i]->switchOn();  //-- "switch on" also acts as "disable".
            ok &= iCanBusSharer[i]->readyToSwitchOn();  //-- "ready to switch on" also acts as "shutdown".
        }

        // Absolute encoders:
        if(value.asString() == "CuiAbsolute")
        {
            int canId = 0;
            int CAN_ID = atoi(nodes[i]->getValue("canId").toString().c_str());
            bool timePassed = false;
            double timeStamp = 0.0;
            double cleaningTime = 0.5; // time to empty the buffer

            ICuiAbsolute* cuiAbsolute;
            nodes[i]->view( cuiAbsolute );

            CD_INFO("Stopping Cui Absolute PIC (ID: %d)\n", CAN_ID );

            if (! cuiAbsolute->stopPublishingMessages() )
                return false;

            yarp::os::Time::delay(0.5);
            timeStamp = yarp::os::Time::now();

            // This part of the code checks if the encoders have stopped sending messages
            while ( !timePassed )
            {
                // -- if it exceeds the timeout (1 secod) ...PASS the test
                if(int(yarp::os::Time::now()-timeStamp)==timeOut)
                {
                    CD_SUCCESS("Time out passed and CuiAbsolute ID (%d) was stopped successfully\n", CAN_ID);
                    timePassed = true;
                }

                unsigned int read;
                bool okRead = iCanBus->canRead(canInputBuffer, 1, &read, true);

                // This line is needed to clear the buffer (old messages that has been received)
                if((yarp::os::Time::now()-timeStamp) < cleaningTime) continue;

                if( !okRead || read == 0 ) continue;              // -- is waiting for recive message

                canId = msg.getId()  & 0x7F;                      // -- if it recive the message, it will get ID
                //CD_DEBUG("Read a message from CuiAbsolute %d\n", canId);

                //printf("timeOut: %d\n", int(yarp::os::Time::now()-timeStamp));
                if(canId == CAN_ID)
                {
                    CD_WARNING("Resending stop message to Cui Absolute PIC (ID: %d)\n", CAN_ID );
                    cuiAbsolute->stopPublishingMessages();
                }
            }
        }
    }

    iCanBufferFactory->destroyBuffer(canInputBuffer);

    //-- Delete the driver objects.
    for(int i=0; i<nodes.size(); i++)
    {
        nodes[i]->close();
        delete nodes[i];
        nodes[i] = 0;
    }

    //-- Clear CAN acceptance filters ('0' = all IDs that were previously set by canIdAdd).
    if (!iCanBus->canIdDelete(0))
    {
        CD_WARNING("CAN filters may be preserved on the next run.\n");
    }

    canBusDevice.close();

    CD_INFO("End.\n");
    return ok;
}

// -----------------------------------------------------------------------------
