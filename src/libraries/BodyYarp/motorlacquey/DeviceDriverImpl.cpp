// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorLacquey.hpp"

// -----------------------------------------------------------------------------
bool teo::MotorLacquey::open(Searchable& config) {

    this->canId = config.check("canId",0,"can bus ID").asInt();
    this->tr = config.check("tr",0,"reduction").asInt();
    this->ptModeMs  = config.check("ptModeMs",0,"ptMode ms").asInt();
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    this->max = 0;
    this->min = 0;
    this->refAcceleration = 0;
    this->refSpeed = 0;
    this->encoder = 0;

    CD_SUCCESS("Created driver with canId %d and tr %f, and all local parameters set to 0.\n",canId,tr);
    return true;
}

// -----------------------------------------------------------------------------
bool teo::MotorLacquey::close() {
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

