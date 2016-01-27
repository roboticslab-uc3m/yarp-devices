// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool teo::CanBusControlboard::open(Searchable& config) {

    //------
    CD_DEBUG("SE ESTÄ EJECUTANDO EL METODO: teo::CanBusControlboard::open(Searchable& config)\n ")
    //------
    CD_DEBUG("INPUT: %s\n", config.find("mode").asString().c_str()); // no le está llegando el modo
    //std::string mode = config.check("mode",Value(DEFAULT_MODE),"position/velocity mode").asString();
    //CD_DEBUG("MODO: %s\n", mode.c_str());
    std::string mode = "velocity";
    int16_t ptModeMs = config.check("ptModeMs",Value(DEFAULT_PT_MODE_MS),"PT mode miliseconds").asInt();

    Bottle ids = config.findGroup("ids").tail();  //-- e.g. 15
    Bottle trs = config.findGroup("trs").tail();  //-- e.g. 160
    Bottle ks = config.findGroup("ks").tail();  //-- e.g. 0.0706

    Bottle maxs = config.findGroup("maxs").tail();  //-- e.g. 360
    Bottle mins = config.findGroup("mins").tail();  //-- e.g. -360
    Bottle refAccelerations = config.findGroup("refAccelerations").tail();  //-- e.g. 0.575437
    Bottle refSpeeds = config.findGroup("refSpeeds").tail();  //-- e.g. 737.2798

    Bottle types = config.findGroup("types").tail();  //-- e.g. 15

    //-- Initialize the CAN device.
    Property canBusOptions;
    canBusOptions.fromString(config.toString());  // canDevice, canBitrate
    canBusOptions.put("device","CanBusHico");
    canBusDevice.open(canBusOptions);
    if( ! canBusDevice.isValid() ){
        CD_ERROR("canBusDevice instantiation not worked.\n");
        return false;
    }
    canBusDevice.view(iCanBus);

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
        Property options;
        options.put("device",types.get(i).asString());  //-- "TechnosoftIpos", "LacqueyFetch"
        options.put("canId",ids.get(i).asInt());
        options.put("tr",trs.get(i).asDouble());
        options.put("min",mins.get(i).asDouble());
        options.put("max",maxs.get(i).asDouble());
        options.put("k",ks.get(i).asDouble());
        options.put("refAcceleration",refAccelerations.get(i).asDouble());
        options.put("refSpeed",refSpeeds.get(i).asDouble());
        options.put("ptModeMs",ptModeMs);
        PolyDriver* driver = new PolyDriver(options);

        //-- Fill a map entry ( drivers.size() if before push_back, otherwise do drivers.size()-1).
        //-- Just "i" if resize already performed.
        idxFromCanId[ ids.get(i).asInt() ] = i;

        //-- Push the motor driver on to the vectors.
        nodes[i] = driver;
        driver->view( iControlLimitsRaw[i] );
        driver->view( iControlModeRaw[i] );
        driver->view( iEncodersTimedRaw[i] );
        driver->view( iPositionControlRaw[i] );
        driver->view( iPositionDirectRaw[i] );
        driver->view( iTorqueControlRaw[i] );
        driver->view( iVelocityControlRaw[i] );
        driver->view( iCanBusSharer[i] );

        //-- Associate absolute encoders to motor drivers
        if( types.get(i).asString() == "CuiAbsolute" ) {
            int driverCanId = ids.get(i).asInt() - 100;
            iCanBusSharer[ idxFromCanId[driverCanId] ]->setIEncodersTimedRawExternal( iEncodersTimedRaw[i] );
        }

        //-- Aditionally sets initial parameters on physical motor drivers.
        iCanBusSharer[i]->setCanBusPtr( iCanBus );
    }

    //-- Set all motor drivers to mode.
    CD_DEBUG("#### DETECCION DEL MODO: %s\n", mode.c_str());
    //std::cout <<"#### DETECCION DEL MODO: "<< mode << "\n";
    if( mode=="position") {
        CD_DEBUG("POSICION ###\n");
        if( ! this->setPositionMode() )
            return false;
    } else if( mode=="velocity") {
        CD_DEBUG("VELOCIDAD ###\n");
        if( ! this->setVelocityMode() )
            return false;
    } else if( mode=="torque") {
        CD_DEBUG("TORQUE ###\n");
        if( ! this->setTorqueMode() )
            return false;
        /////////////////
        else CD_DEBUG("HA ENTRADO EN EL MODO TORQUE CORRECTAMENTE\n");
    } else {
        CD_ERROR("Not prepared for initializing in mode %s.\n",mode.c_str());
        return false;
    }

    //-- Check the status of each driver.
    std::vector<int> tmp( nodes.size() );
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

    if( config.check("home") ) {
        CD_DEBUG("Moving motors to zero.\n");
        for(int i=0; i<nodes.size(); i++)
        {
            if ( ! iPositionControlRaw[i]->positionMoveRaw(0,0) )
                return false;
        }
        for(int i=0; i<nodes.size(); i++)
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

bool teo::CanBusControlboard::close() {

    //-- Stop the read thread.
    this->Thread::stop();

    //-- Disable and shutdown the physical drivers.
    bool ok = true;
    for(int i=0; i<nodes.size(); i++)
    {
        ok &= iCanBusSharer[i]->switchOn();  //-- "switch on" also acts as "disable".

        ok &= iCanBusSharer[i]->readyToSwitchOn();  //-- "ready to switch on" also acts as "shutdown".
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

