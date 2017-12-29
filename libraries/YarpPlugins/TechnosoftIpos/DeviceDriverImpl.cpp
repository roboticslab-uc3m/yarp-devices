// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// -----------------------------------------------------------------------------
bool roboticslab::TechnosoftIpos::open(yarp::os::Searchable& config)
{

    // -- .ini parameters (in order)
    this->canId = config.check("canId",0,"can bus ID").asInt();    
    this->max = config.check("max",0,"max").asDouble();
    this->min = config.check("min",0,"min").asDouble();
    this->maxVel = config.check("maxVel",1000,"maxVel").asDouble();
    this->minVel = config.check("minVel",0,"minVel").asDouble();
    this->tr = config.check("tr",0,"reduction").asDouble();
    this->refAcceleration = config.check("refAcceleration",0,"ref acceleration").asDouble();
    this->refSpeed = config.check("refSpeed",0,"ref speed").asDouble();
    this->encoderPulses = config.check("encoderPulses",0,"encoderPulses").asInt();

    // -- other parameters...
    this->k = config.check("k",0,"motor constant").asDouble();
    this->ptModeMs  = config.check("ptModeMs",0,"ptMode ms").asInt();
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    this->encoder = 0;    
    this->refTorque = 0;
    this->refVelocity = 0; // if you want to test.. put 0.1

    yarp::os::Value vCanBufferFactory = config.check("canBufferFactory", 0, "");

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
    if( 0 == this->encoderPulses )
    {
        CD_ERROR("Could not create TechnosoftIpos with encoderPulses 0\n");
        return false;
    }
    if( !vCanBufferFactory.isBlob() || vCanBufferFactory.asBlobLength() != sizeof(yarp::dev::ICanBufferFactory) )
    {
        CD_ERROR("Could not create TechnosoftIpos with null or corrupt ICanBufferFactory handle\n");
        return false;
    }

    iCanBufferFactory = reinterpret_cast<yarp::dev::ICanBufferFactory *>(const_cast<char *>(vCanBufferFactory.asBlob()));
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

