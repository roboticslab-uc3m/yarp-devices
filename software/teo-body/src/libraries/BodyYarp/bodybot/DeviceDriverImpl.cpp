// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::BodyBot::open(Searchable& config) {

    std::string mode = config.check("mode",Value(DEFAULT_MODE),"position/velocity mode").asString();

    std::string canDevicePath = config.check("canDevice",Value(DEFAULT_CAN_DEVICE),"CAN device path").asString();
    int canBitrate = config.check("canBitrate",Value(DEFAULT_CAN_BITRATE),"CAN bitrate").asInt();
    int16_t ptModeMs = config.check("ptModeMs",Value(DEFAULT_PT_MODE_MS),"PT mode miliseconds").asInt();

    Bottle ids = config.findGroup("ids").tail();  //-- e.g. 15
    Bottle trs = config.findGroup("trs").tail();  //-- e.g. 160

    Bottle maxs = config.findGroup("maxs").tail();  //-- e.g. 360
    Bottle mins = config.findGroup("mins").tail();  //-- e.g. -360
    Bottle refAccelerations = config.findGroup("refAccelerations").tail();  //-- e.g. 0.575437
    Bottle refSpeeds = config.findGroup("refSpeeds").tail();  //-- e.g. 737.2798

    Bottle types = config.findGroup("types").tail();  //-- e.g. 15

    //-- Initialize the CAN device (i.e. /dev/can0, set in DEFAULT_CAN_DEVICE).
    if( ! canDevice.init(canDevicePath, canBitrate) )
        return false;

    //-- Start the reading thread (required for checkMotionDoneRaw).
    this->Thread::start();

    //-- Populate the motor drivers vector.
    drivers.resize( ids.size() );
    iControlLimitsRaw.resize( drivers.size() );
    iControlModeRaw.resize( drivers.size() );
    iEncodersTimedRaw.resize( drivers.size() );
    iPositionControlRaw.resize( drivers.size() );
    iPositionDirectRaw.resize( drivers.size() );
    iTorqueControlRaw.resize( drivers.size() );
    iVelocityControlRaw.resize( drivers.size() );
    iCanBusSharer.resize( drivers.size() );
    for(int i=0; i<drivers.size(); i++)
    {
        //-- Create motor driver object with a pointer to the CAN device, its id and tr (these are locally stored parameters).
        Property options;
        options.put("device",types.get(i).asString());  //-- "motoripos", "motorlackey"
        options.put("canId",ids.get(i).asInt());
        options.put("tr",trs.get(i).asInt());
        options.put("ptModeMs",ptModeMs);
        PolyDriver* driver = new PolyDriver(options);

        //-- Fill a map entry ( drivers.size() if before push_back, otherwise do drivers.size()-1).
        //-- Just "i" if resize already performed.
        idxFromCanId[ ids.get(i).asInt() ] = i;

        //-- Push the motor driver on to the vectors.
        drivers[i] = driver;
        driver->view( iControlLimitsRaw[i] );
        driver->view( iControlModeRaw[i] );
        driver->view( iEncodersTimedRaw[i] );
        driver->view( iPositionControlRaw[i] );
        driver->view( iPositionDirectRaw[i] );
        driver->view( iTorqueControlRaw[i] );
        driver->view( iVelocityControlRaw[i] );
        driver->view( iCanBusSharer[i] );

        iCanBusSharer[i]->setCanBusPtr( &canDevice );
    }

    //-- Set initial parameters on physical motor drivers.
    for(int i=0; i<drivers.size(); i++)
    {
        if ( ! this->setRefAcceleration( i, refAccelerations.get(i).asDouble() ) )
            return false;
        if ( ! this->setRefSpeed( i, refSpeeds.get(i).asDouble() ) )
            return false;
        if ( ! this->setLimits( i, mins.get(i).asDouble(), maxs.get(i).asDouble() ) )
            return false;
    }

    //-- Set all motor drivers to mode.
    if( mode=="position") {
        if( ! this->setPositionMode() )
            return false;
    } else if( mode=="velocity") {
        if( ! this->setVelocityMode() )
            return false;
    } else if( mode=="torque") {
        if( ! this->setTorqueMode() )
            return false;
    } else {
        CD_ERROR("Not prepared for initializing in mode %s.\n",mode.c_str());
        return false;
    }

    //-- Check the status of each driver.
    std::vector<int> tmp( drivers.size() );
    this->getControlModes( tmp.data() );

    //-- Initialize the drivers: start (0.1) ready (0.1) on (2) enable. Wait between each step.
    for(int i=0; i<drivers.size(); i++)
    {
        if( ! iCanBusSharer[i]->start() )
            return false;
    }
    yarp::os::Time::delay(0.1);
    for(int i=0; i<drivers.size(); i++)
    {
        if( ! iCanBusSharer[i]->readyToSwitchOn() )
            return false;
    }
    yarp::os::Time::delay(0.1);
    for(int i=0; i<drivers.size(); i++)
    {
        if( ! iCanBusSharer[i]->switchOn() )
            return false;
    }
    yarp::os::Time::delay(2);
    for(int i=0; i<drivers.size(); i++)
    {
        if( ! iCanBusSharer[i]->enable() )
            return false;
    }

    if( config.check("home") ) {
        CD_DEBUG("Moving motors to zero.\n");
        for(int i=0; i<drivers.size(); i++)
        {
            if ( ! iPositionControlRaw[i]->positionMoveRaw(0,0) )
                return false;
        }
        for(int i=0; i<drivers.size(); i++)
        {
            bool motionDone = false;
            while( ! motionDone ) {
                yarp::os::Time::delay(0.5);  //-- [s]
                CD_DEBUG("Moving %d to zero...\n",i);
                if( ! iPositionControlRaw[i]->checkMotionDoneRaw(0,&motionDone) )
                    return false;
            }
        }
        CD_DEBUG("Moved motors to zero.\n");
    }

    if( config.check("reset") ) {
        CD_DEBUG("Forcing encoders to zero.\n");
        if ( ! this->resetEncoders() )
            return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::close() {

    //-- Stop the read thread.
    this->Thread::stop();

    //-- Disable and shutdown the physical drivers.
    bool ok = true;
    for(int i=0; i<drivers.size(); i++)
    {
        ok &= iCanBusSharer[i]->switchOn();  //-- "switch on" also acts as "disable".

        ok &= iCanBusSharer[i]->readyToSwitchOn();  //-- "ready to switch on" also acts as "shutdown".
    }

    //-- Delete the driver objects.
    for(int i=0; i<drivers.size(); i++)
    {
        delete drivers[i];
        drivers[i] = 0;
    }

    canDevice.close();
    CD_INFO("End.\n");
    return ok;
}

// -----------------------------------------------------------------------------

