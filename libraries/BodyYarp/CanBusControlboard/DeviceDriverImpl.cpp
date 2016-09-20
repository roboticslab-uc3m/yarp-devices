// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"
#include "CuiAbsolute/CuiAbsolute.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::CanBusControlboard::open(yarp::os::Searchable& config)
{

    std::string mode = config.check("mode",yarp::os::Value("position"),"position/velocity mode").asString();
    int16_t ptModeMs = config.check("ptModeMs",yarp::os::Value(DEFAULT_PT_MODE_MS),"PT mode miliseconds").asInt();


    yarp::os::Bottle ids = config.findGroup("ids").tail();  //-- e.g. 15
    yarp::os::Bottle trs = config.findGroup("trs").tail();  //-- e.g. 160
    yarp::os::Bottle ks = config.findGroup("ks").tail();  //-- e.g. 0.0706

    yarp::os::Bottle maxs = config.findGroup("maxs").tail();  //-- e.g. 360
    yarp::os::Bottle mins = config.findGroup("mins").tail();  //-- e.g. -360
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
    iControlLimitsRaw.resize( nodes.size() );
    iControlModeRaw.resize( nodes.size() );
    iEncodersTimedRaw.resize( nodes.size() );
    iPositionControlRaw.resize( nodes.size() );
    iPositionDirectRaw.resize( nodes.size() );
    iTorqueControlRaw.resize( nodes.size() );
    iVelocityControlRaw.resize( nodes.size() );
    iCanBusSharer.resize( nodes.size() );
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
        nodes[i] = device;  // -- device es un puntero que guarda la dirección de un objeto PolyDriver
        // -- nodes es un vector de punteros que apuntará al device
        device->view( iControlLimitsRaw[i] );
        device->view( iControlModeRaw[i] );
        device->view( iEncodersTimedRaw[i] );
        device->view( iPositionControlRaw[i] );
        device->view( iPositionDirectRaw[i] );
        device->view( iTorqueControlRaw[i] );
        device->view( iVelocityControlRaw[i] );
        device->view( iCanBusSharer[i] );   // -- si el device es un Cui, este podrá "ver" las funciones programadas en iCanBusSharer (funciones que hemos añadido al encoder).
        // -- estas funciones se encuentran implementadas en el cpp correspondiente "ICanBusSharerImpl.cpp", por lo tanto le da la funcionalidad que deseamos

        //-- Pass CAN bus pointer to CAN node
        iCanBusSharer[i]->setCanBusPtr( iCanBus );

        //-- Associate absolute encoders to motor drivers
        if( types.get(i).asString() == "CuiAbsolute" )
        {           
            int driverCanId = ids.get(i).asInt() - 100;  //-- \todo{Document the dangers: ID must be > 100, driver must be instanced.}

            CD_INFO("Sending \"Start Continuous Publishing\" message to Cui Absolute PIC (ID: %i)\n", ids.get(i).asInt());

            // Configuring Cui Absolute
            CuiAbsolute* cuiAbsolute;
            device->view( cuiAbsolute );

            yarp::os::Time::delay(0.5);            
            cuiAbsolute->startContinuousPublishing(0); // startContinuousPublishing(delay)

            (iCanBusSharer[ idxFromCanId[driverCanId] ]->setIEncodersTimedRawExternal( iEncodersTimedRaw[i] ));
        }

        //-- Set initial parameters on physical motor drivers.

        if ( ! iPositionControlRaw[i]->setRefAccelerationRaw( 0, refAccelerations.get(i).asDouble() ) )
            return false;

        if ( ! iPositionControlRaw[i]->setRefSpeedRaw( 0, refSpeeds.get(i).asDouble() ) )
            return false;

        if ( ! iControlLimitsRaw[i]->setLimitsRaw( 0, mins.get(i).asDouble(), maxs.get(i).asDouble() ) )
            return false;

    } // -- for(int i=0; i<nodes.size(); i++)

    //-- Set all motor drivers to mode.

    if( mode=="position")
    {
        if( ! this->setPositionMode() )
            return false;
    }
    else if( mode=="velocity")
    {
        if( ! this->setVelocityMode() )
            return false;
    }
    else if( mode=="torque")
    {
        if( ! this->setTorqueMode() )
            return false;
    }
    else
    {
        CD_ERROR("Not prepared for initializing in mode %s.\n",mode.c_str());
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

    if( config.check("home") )
    {
        CD_DEBUG("Moving motors to zero.\n");
        for(int i=0; i<nodes.size(); i++)
        {
            if ( ! iPositionControlRaw[i]->positionMoveRaw(0,0) )
                return false;
        }
        for(int i=0; i<nodes.size(); i++)
        {
            bool motionDone = false;
            while( ! motionDone )
            {
                yarp::os::Time::delay(0.5);  //-- [s]
                CD_DEBUG("Moving %d to zero...\n",i);
                if( ! iPositionControlRaw[i]->checkMotionDoneRaw(0,&motionDone) )
                    return false;
            }
        }
        CD_DEBUG("Moved motors to zero.\n");
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

bool teo::CanBusControlboard::close()
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
        if(value.asString() == "TechnosoftIpos"){
            CD_INFO("Stopping Driver (ID: %s)\n", nodes[i]->getValue("canId").toString().c_str());
            ok &= iCanBusSharer[i]->switchOn();  //-- "switch on" also acts as "disable".
            ok &= iCanBusSharer[i]->readyToSwitchOn();  //-- "ready to switch on" also acts as "shutdown".
        }

        // Absolute encoders:
        if(value.asString() == "CuiAbsolute"){

            int canId = 0;
            int CAN_ID = atoi(nodes[i]->getValue("canId").toString().c_str());
            bool timePassed = false;
            double timeStamp = 0.0;
            double cleaningTime = 0.5; // time to empty the buffer

            CuiAbsolute* cuiAbsolute;
            nodes[i]->view( cuiAbsolute );

                CD_INFO("Stopping Cui Absolute PIC (ID: %d)\n", CAN_ID );
                cuiAbsolute->stopPublishingMessages();

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
                    if(canId == CAN_ID)  {                        
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

