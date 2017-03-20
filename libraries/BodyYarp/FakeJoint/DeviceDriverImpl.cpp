// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// -----------------------------------------------------------------------------
bool teo::FakeJoint::open(yarp::os::Searchable& config)
{

    this->canId = config.check("canId",0,"can bus ID").asInt();
    this->tr = config.check("tr",0,"reduction").asInt();
    this->ptModeMs  = config.check("ptModeMs",0,"ptMode ms").asInt();
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    this->max = config.check("max",0,"max").asDouble();
    this->min = config.check("min",0,"min").asDouble();
    this->maxVel = config.check("maxVel",1000,"maxVel").asDouble();
    this->minVel = config.check("minVel",0,"minVel").asDouble();
    this->refAcceleration = 0;
    this->refSpeed = 0;
    this->encoder = 0;

    CD_SUCCESS("Created FakeJoint with canId %d and tr %f, and all local parameters set to 0.\n",canId,tr);
    return true;
}

// -----------------------------------------------------------------------------
bool teo::FakeJoint::close()
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

