// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// -----------------------------------------------------------------------------
bool roboticslab::TechnosoftIpos::open(yarp::os::Searchable& config)
{

    // -- .ini parameters (in order)
    this->canId = config.check("canId",yarp::os::Value(0),"can bus ID").asInt32();    
    this->max = config.check("max",yarp::os::Value(0),"max (meters or degrees)").asFloat64();
    this->min = config.check("min",yarp::os::Value(0),"min (meters or degrees)").asFloat64();
    this->maxVel = config.check("maxVel",yarp::os::Value(10),"maxVel (meters/second or degrees/second)").asFloat64();
    this->tr = config.check("tr",yarp::os::Value(0),"reduction").asFloat64();
    this->refAcceleration = config.check("refAcceleration",yarp::os::Value(0),"ref acceleration (meters/second^2 or degrees/second^2)").asFloat64();
    this->refSpeed = config.check("refSpeed",yarp::os::Value(0),"ref speed (meters/second or degrees/second)").asFloat64();
    this->encoderPulses = config.check("encoderPulses",yarp::os::Value(0),"encoderPulses").asInt32();

    // -- other parameters...
    this->k = config.check("k",yarp::os::Value(0),"motor constant").asFloat64();
    this->ptModeMs  = config.check("ptModeMs",yarp::os::Value(0),"ptMode (milliseconds)").asInt32();
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    this->encoder = 0;    
    this->refTorque = 0;
    this->refVelocity = 0; // if you want to test.. put 0.1
    this->refCurrent = 0;

    yarp::os::Value vCanBufferFactory = config.check("canBufferFactory", yarp::os::Value(0), "");

    if( 0 == this->canId )
    {
        CD_ERROR("Could not create TechnosoftIpos with canId 0\n");
        return false;
    }
    if( this->min >= this->max )
    {
        CD_ERROR("Could not create TechnosoftIpos with min >= max\n");
        return false;
    }
    if( 0 == this->maxVel )
    {
        CD_ERROR("Could not create TechnosoftIpos with maxVel 0\n");
        return false;
    }
    if( 0 == this->tr )
    {
        CD_ERROR("Could not create TechnosoftIpos with tr 0\n");
        return false;
    }
    if( 0 == this->refAcceleration )
    {
        CD_ERROR("Could not create TechnosoftIpos with refAcceleration 0\n");
        return false;
    }
    if( 0 == this->refSpeed )
    {
        CD_ERROR("Could not create TechnosoftIpos with refSpeed 0\n");
        return false;
    }
    if( this->refSpeed > this->maxVel )
    {
        CD_ERROR("Could not create TechnosoftIpos with refSpeed > maxVel\n");
        return false;
    }
    if( 0 == this->encoderPulses )
    {
        CD_ERROR("Could not create TechnosoftIpos with encoderPulses 0\n");
        return false;
    }
    if( !vCanBufferFactory.isBlob() )
    {
        CD_ERROR("Could not create TechnosoftIpos with null or corrupt ICanBufferFactory handle\n");
        return false;
    }

    iCanBufferFactory = *reinterpret_cast<yarp::dev::ICanBufferFactory **>(const_cast<char *>(vCanBufferFactory.asBlob()));
    canOutputBuffer = iCanBufferFactory->createBuffer(1);

    CD_SUCCESS("Created TechnosoftIpos with canId %d, tr %f, k %f, refAcceleration %f, refSpeed %f, encoderPulses %d and all local parameters set to 0.\n",
               canId,tr,k,refAcceleration,refSpeed,encoderPulses);
    return true;
}

// -----------------------------------------------------------------------------
bool roboticslab::TechnosoftIpos::close()
{
    CD_INFO("\n");
    iCanBufferFactory->destroyBuffer(canOutputBuffer);
    return true;
}

// -----------------------------------------------------------------------------

