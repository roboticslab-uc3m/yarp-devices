// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CanBusControlboard::open(yarp::os::Searchable& config)
{

    std::string mode = config.check("mode",yarp::os::Value("position"),"position/velocity mode").asString();
    int16_t ptModeMs = config.check("ptModeMs",yarp::os::Value(DEFAULT_PT_MODE_MS),"PT mode miliseconds").asInt();
    int timeCuiWait  = config.check("waitEncoder", yarp::os::Value(DEFAULT_TIME_TO_WAIT_CUI), "CUI timeout seconds").asInt();



    yarp::os::Bottle ids = config.findGroup("ids").tail();  //-- e.g. 15
    yarp::os::Bottle trs = config.findGroup("trs").tail();  //-- e.g. 160
    yarp::os::Bottle ks = config.findGroup("ks").tail();  //-- e.g. 0.0706

    yarp::os::Bottle maxs = config.findGroup("maxs").tail();  //-- e.g. 360
    yarp::os::Bottle mins = config.findGroup("mins").tail();  //-- e.g. -360
    yarp::os::Bottle maxVels = config.findGroup("maxVels").tail();  //-- e.g. 1000
    yarp::os::Bottle minVels = config.findGroup("minVels").tail();  //-- e.g. 0
    yarp::os::Bottle refAccelerations = config.findGroup("refAccelerations").tail();  //-- e.g. 0.575437
    yarp::os::Bottle refSpeeds = config.findGroup("refSpeeds").tail();  //-- e.g. 737.2798

    yarp::os::Bottle types = config.findGroup("types").tail();  //-- e.g. 15

    //-- Initialize the CAN device.
    yarp::os::Property canBusOptions;
    canBusOptions.fromString(config.toString());  // canDevice, canBitrate
    canBusOptions.put("device","CanBusHico");
    canBusDevice.open(canBusOptions);
    if( ! canBusDevice.isValid() )
    {
        CD_ERROR("canBusDevice instantiation not worked.\n");
        return false;
    }
    canBusDevice.view(iCanBus);

    //-- We test if we recive messages of the Cui (in the start and in the end)
    struct can_msg buffer;

    //-- Start the reading thread (required for checkMotionDoneRaw).
    this->Thread::start();

    //-- Populate the CAN nodes vector.
    nodes.resize( ids.size() );
    iControlLimits2Raw.resize( nodes.size() );
    iControlMode2Raw.resize( nodes.size() );
    iEncodersTimedRaw.resize( nodes.size() );
    iPositionControl2Raw.resize( nodes.size() );
    iPositionDirectRaw.resize( nodes.size() );
    iTorqueControlRaw.resize( nodes.size() );
    iCanBusSharer.resize( nodes.size() );

    iInteractionModeRaw.resize( nodes.size() );
    iVelocityControl2Raw.resize( nodes.size() ); // -- new

    for(int i=0; i<nodes.size(); i++)
    {
        if(types.get(i).asString() == "")
            CD_WARNING("Argument \"types\" empty at %d.\n",i);

        //-- Create CAN node objects with a pointer to the CAN device, its id and tr (these are locally stored parameters).
        yarp::os::Property options;
        options.put("device",types.get(i).asString());  //-- "TechnosoftIpos", "LacqueyFetch", "CuiAbsolute"
        options.put("canId",ids.get(i).asInt());
        options.put("tr",trs.get(i).asDouble());
        options.put("min",mins.get(i).asDouble());
        options.put("max",maxs.get(i).asDouble());
        options.put("minVel",minVels.get(i).asDouble());
        options.put("maxVel",maxVels.get(i).asDouble());
        options.put("k",ks.get(i).asDouble());
        options.put("refAcceleration",refAccelerations.get(i).asDouble());
        options.put("refSpeed",refSpeeds.get(i).asDouble());
        options.put("ptModeMs",ptModeMs);

        // -- Configuramos todos los dispositivos (TechnosoftIpos, LacqueyFetch, CuiAbsolute)
        yarp::dev::PolyDriver* device = new yarp::dev::PolyDriver(options);

        //-- Fill a map entry ( drivers.size() if before push_back, otherwise do drivers.size()-1).
        //-- Just "i" if resize already performed.
        idxFromCanId[ ids.get(i).asInt() ] = i;

        //-- Push the motor driver and other devices (CuiAbsolute) on to the vectors.
        nodes[i] = device;

        //-- View interfaces
        if( !device->view( iControlLimits2Raw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iControlLimits2Raw interface\n");
            return false;
        }

        if( !device->view( iControlMode2Raw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iControlMode2Raw interface\n");
            return false;
        }

        if( !device->view( iEncodersTimedRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iEncodersTimedRaw interface\n");
            return false;
        }

        if( !device->view( iPositionControl2Raw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iPositionControl2Raw interface\n");
            return false;
        }

        if( !device->view( iPositionDirectRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iPositionDirectRaw interface\n");
            return false;
        }

        if( !device->view( iTorqueControlRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iTorqueControlRaw interface\n");
            return false;
        }

        if( !device->view( iVelocityControl2Raw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iVelocityControl2Raw interface\n");
            return false;
        }
        // -- si el device es un Cui, este podrá "ver" las funciones programadas en iCanBusSharer (funciones que hemos añadido al encoder).
        // -- estas funciones se encuentran implementadas en el cpp correspondiente "ICanBusSharerImpl.cpp", por lo tanto le da la funcionalidad que deseamos
        if(! device->view( iCanBusSharer[i] ))
        {
            CD_ERROR("[error] Problems acquiring iCanBusSharer interface\n");
            return false;
        }

        if(! device->view( iInteractionModeRaw[i] ))
        {
            CD_ERROR("[error] Problems acquiring iInteractionModeRaw interface\n");
            return false;
        }

        //-- Pass CAN bus pointer to CAN node
        iCanBusSharer[i]->setCanBusPtr( iCanBus );               

        //-- DRIVERS
        if(types.get(i).asString() == "TechnosoftIpos")
        {
            //-- Set initial parameters on physical motor drivers.

            if ( ! iPositionControl2Raw[i]->setRefAccelerationRaw( 0, refAccelerations.get(i).asDouble() ) )
                return false;

            if ( ! iPositionControl2Raw[i]->setRefSpeedRaw( 0, refSpeeds.get(i).asDouble() ) )
                return false;

            if ( ! iControlLimits2Raw[i]->setLimitsRaw( 0, mins.get(i).asDouble(), maxs.get(i).asDouble() ) )
                return false;
        }

        //-- Associate absolute encoders to motor drivers
        if( types.get(i).asString() == "CuiAbsolute" )
        {
            int driverCanId = ids.get(i).asInt() - 100;  //-- \todo{Document the dangers: ID must be > 100, driver must be instanced.}

            CD_INFO("Sending \"Start Continuous Publishing\" message to Cui Absolute (PIC ID: %d)\n", ids.get(i).asInt());

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
                        CD_ERROR("Time out passed and CuiAbsolute ID (%d) doesn't respond\n", ids.get(i).asInt() );
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
                    CD_ERROR("Cui Absolute (PIC ID: %d) doesn't respond. Try using --externalEncoderWait [seconds] parameter with timeout higher than 0 \n", ids.get(i).asInt());
                    return false;
                }
            }
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
    if( config.check("home") )
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
                    if ( ! iPositionControl2Raw[i]->positionMoveRaw(0,0) )
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
                if( ! iPositionControl2Raw[i]->checkMotionDoneRaw(0,&motionDone) )
                    return false;
                if(!motionDone)
                    CD_WARNING("Test motion fail (ID:%s) \n", nodes[i]->getValue("canId").toString().c_str());
            }
        }

        CD_DEBUG("Moved motors to zero.\n");
        yarp::os::Time::delay(1);
    }

    if( config.check("reset") )
    {
        CD_DEBUG("Forcing encoders to zero.\n");
        if ( ! this->resetEncoders() )
            return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::close()
{
    int ret = 0;
    double timeOut = 1; // timeout (1 secod)
    struct can_msg buffer;


    //-- Stop the read thread.
    this->Thread::stop();

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

                ret = iCanBus->read_timeout(&buffer,1);         // -- return value of message with timeout of 1 [ms]

                // This line is needed to clear the buffer (old messages that has been received)
                if((yarp::os::Time::now()-timeStamp) < cleaningTime) continue;

                if( ret <= 0 ) continue;                        // -- is waiting for recive message
                canId = buffer.id  & 0x7F;                      // -- if it recive the message, it will get ID
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

    //-- Delete the driver objects.
    for(int i=0; i<nodes.size(); i++)
    {
        delete nodes[i];
        nodes[i] = 0;
    }

    canBusDevice.close();
    CD_INFO("End.\n");
    return ok;
}

// -----------------------------------------------------------------------------

