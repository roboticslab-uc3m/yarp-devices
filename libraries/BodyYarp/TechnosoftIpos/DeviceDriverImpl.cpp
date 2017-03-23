// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// -----------------------------------------------------------------------------
bool teo::TechnosoftIpos::open(yarp::os::Searchable& config)
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

    // -- other parameters...
    this->k = config.check("k",0,"motor constant").asDouble();
    this->ptModeMs  = config.check("ptModeMs",0,"ptMode ms").asInt();
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    this->encoder = 0;    
    this->refTorque = 0;
    this->refVelocity = 0.1;


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

    CD_SUCCESS("Created TechnosoftIpos with canId %d, tr %f, k %f, refAcceleration %f, refSpeed %f, and all local parameters set to 0.\n",
               canId,tr,k,refSpeed,refAcceleration);
    return true;
}

// -----------------------------------------------------------------------------
bool teo::TechnosoftIpos::close()
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

