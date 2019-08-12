// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

// -----------------------------------------------------------------------------
bool roboticslab::LacqueyFetch::open(yarp::os::Searchable& config)
{
    this->canId = config.check("canId",yarp::os::Value(0),"can bus ID").asInt32();
    this->tr = config.check("tr",yarp::os::Value(0),"reduction").asInt32();
    this->max = config.check("max",yarp::os::Value(0),"max").asFloat64();
    this->min = config.check("min",yarp::os::Value(0),"min").asFloat64();
    this->refAcceleration = 0;
    this->refSpeed = 0;
    this->encoder = 0;

    CD_SUCCESS("Created LacqueyFetch with canId %d and tr %f, and all local parameters set to 0.\n",canId,tr);
    return true;
}

// -----------------------------------------------------------------------------
bool roboticslab::LacqueyFetch::close()
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------
