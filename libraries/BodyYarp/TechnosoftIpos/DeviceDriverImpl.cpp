// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// -----------------------------------------------------------------------------
bool teo::TechnosoftIpos::open(Searchable& config) {

    yarp::os::Value canPtr = config.check("canPtr",Value(this->canDevicePtr,sizeof(CanBusHico*)),"can bus ptr");
    this->canDevicePtr = (CanBusHico*) (canPtr.asBlob());
    this->canId = config.check("canId",-1,"can bus ID").asInt();
    this->tr = config.check("tr",-1,"reduction").asDouble();
    this->k = config.check("k",-1,"motor constant").asDouble();
    this->ptModeMs  = config.check("ptModeMs",-1,"ptMode ms").asInt();
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    this->max = 0;
    this->min = 0;
    this->refAcceleration = 0;
    this->refSpeed = 0;
    this->encoder = 0;

    if( -1 == this->canId ) {
        CD_ERROR("Could not create TechnosoftIpos with canId -1\n");
        return false;
    }
    if( -1 == this->tr ) {
        CD_ERROR("Could not create TechnosoftIpos with tr -1\n");
        return false;
    }

    CD_SUCCESS("Created TechnosoftIpos with canId %d, tr %f, k %f, and all local parameters set to 0.\n",canId,tr,k);
    return true;
}

// -----------------------------------------------------------------------------
bool teo::TechnosoftIpos::close() {
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

