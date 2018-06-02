// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// -----------------------------------------------------------------------------
bool roboticslab::FakeJoint::open(yarp::os::Searchable& config)
{

    this->canId = config.check("canId",yarp::os::Value(0),"can bus ID").asInt();
    this->tr = config.check("tr",yarp::os::Value(0),"reduction").asInt();
    this->ptModeMs  = config.check("ptModeMs",yarp::os::Value(0),"ptMode (milliseconds)").asInt();
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    this->max = config.check("max",yarp::os::Value(0),"max (meters or degrees)").asDouble();
    this->min = config.check("min",yarp::os::Value(0),"min (meters or degrees)").asDouble();
    this->maxVel = config.check("maxVel",yarp::os::Value(1000),"maxVel (meters/second or degrees/second)").asDouble();
    this->minVel = config.check("minVel",yarp::os::Value(0),"minVel (meters/second or degrees/second)").asDouble();
    this->refAcceleration = 0;
    this->refSpeed = 0;
    this->encoder = 0;

    CD_SUCCESS("Created FakeJoint with canId %d and tr %f, and all local parameters set to 0.\n",canId,tr);
    return true;
}

// -----------------------------------------------------------------------------
bool roboticslab::FakeJoint::close()
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

