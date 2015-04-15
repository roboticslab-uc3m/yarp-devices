// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// -----------------------------------------------------------------------------
bool teo::TechnosoftIpos::open(Searchable& config) {

    this->canId = config.check("canId",0,"can bus ID").asInt();
    this->tr = config.check("tr",0,"reduction").asDouble();
    this->k = config.check("k",0,"motor constant").asDouble();
    this->ptModeMs  = config.check("ptModeMs",0,"ptMode ms").asInt();
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    this->max = 0;
    this->min = 0;
    this->refAcceleration = 0;
    this->refSpeed = 0;
    this->encoder = 0;

    CD_SUCCESS("Created TechnosoftIpos with canId %d, tr %f, k %f, and all local parameters set to 0.\n",canId,tr,k);
    return true;
}

// -----------------------------------------------------------------------------
bool teo::TechnosoftIpos::close() {
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

