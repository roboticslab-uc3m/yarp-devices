// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::BodyBot::open(Searchable& config) {

    std::string mode = config.check("mode",Value(DEFAULT_MODE),"position/velocity mode").asString();

    std::string canDevicePath = config.check("canDevice",Value(DEFAULT_CAN_DEVICE),"CAN device path").asString();
    int canBitrate = config.check("canBitrate",Value(DEFAULT_CAN_BITRATE),"CAN bitrate").asInt();
    this->ptModeMs = config.check("ptModeMs",Value(DEFAULT_PT_MODE_MS),"PT mode miliseconds").asInt();

    Bottle ids = config.findGroup("ids").tail();  //-- e.g. 15
    Bottle trs = config.findGroup("trs").tail();  //-- e.g. 160

    Bottle maxs = config.findGroup("maxs").tail();  //-- e.g. 360
    Bottle mins = config.findGroup("mins").tail();  //-- e.g. -360
    Bottle refAccelerations = config.findGroup("refAccelerations").tail();  //-- e.g. 0.575437
    Bottle refSpeeds = config.findGroup("refSpeeds").tail();  //-- e.g. 737.2798
    Bottle initPoss = config.findGroup("initPoss").tail();  //-- e.g. 0 0 0 45

    //-- Populate the motor drivers vector.
    for(int i=0; i<ids.size(); i++)
    {
        //-- Create motor driver object with a pointer to the CAN device, its id and tr (these are locally stored parameters).
        //MotorIpos* driver = new MotorIpos( &canDevice, ids.get(i).asInt(), trs.get(i).asDouble(), ptModeMs );

        PolyDriver* driver;

        //-- Fill a map entry ( drivers.size() if before push_back, otherwise do drivers.size()-1).
        idxFromCanId[ ids.get(i).asInt() ] = drivers.size();

        //-- Push the motor driver on to the drivers vector.
        drivers.push_back(driver);
    }

    //-- Initialize the CAN device (i.e. /dev/can0, set in DEFAULT_CAN_DEVICE).
    if( ! canDevice.init(canDevicePath, canBitrate) )
        return false;

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
    for(int i=0; i<drivers.size(); i++)
    {
        Time::delay(0.2);
        std::vector<int> tmp(drivers.size());
        getControlModes(&(tmp[0]));
    }

    Time::delay(1);

    //-- Initialize the drivers: start (0.1) ready (0.1) on (2) enable. Wait between each step.
    /*for(int i=0; i<drivers.size(); i++)
    {
        if ( ! drivers[i]->start() )
            return false;
    }
    yarp::os::Time::delay(0.1);

    for(int i=0; i<drivers.size(); i++)
    {
        if ( ! drivers[i]->readyToSwitchOn() )
            return false;
    }
    yarp::os::Time::delay(0.1);

    for(int i=0; i<drivers.size(); i++)
    {
        if ( ! drivers[i]->switchOn() )
            return false;
    }
    yarp::os::Time::delay(2);

    for(int i=0; i<drivers.size(); i++)
    {
        if ( ! drivers[i]->enable() )
            return false;
    }*/

    yarp::os::Time::delay(1);
    if( ! config.findGroup("initPoss").isNull() ) {
        CD_DEBUG("Setting initPoss.\n");
        for(int i=0; i<initPoss.size(); i++)
        {
            if ( ! this->setEncoder(i,initPoss.get(i).asDouble()) )
                return false;
        }
    }

    //-- Start the thread.
    this->Thread::start();

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::close() {

    //-- Stop the read thread.
    this->Thread::stop();

    //-- Disable and shutdown the physical drivers.
    bool ok = true;
    /*for(int i=0; i<drivers.size(); i++)
    {
        ok &= drivers[i]->switchOn();  //-- "switch on" also acts as "disable".
    }
    for(int i=0; i<drivers.size(); i++)
    {
        ok &= drivers[i]->readyToSwitchOn();  //-- "ready to switch on" also acts as "shutdown".
    }*/

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

