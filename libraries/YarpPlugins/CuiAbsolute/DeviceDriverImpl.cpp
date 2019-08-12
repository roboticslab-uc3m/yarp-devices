// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

// -----------------------------------------------------------------------------
bool roboticslab::CuiAbsolute::open(yarp::os::Searchable& config)
{
    this->canId = config.check("canId",yarp::os::Value(0),"can bus ID").asInt32();
    this->tr = config.check("tr",yarp::os::Value(1),"reduction").asInt32();
    this->targetReached = false;
    this->max = 0;
    this->min = 0;
    this->refAcceleration = 0;
    this->refSpeed = 0;
    this->encoder = sqrt (-1);  // NaN \todo{Investigate, debug and document the dangers of this use of NaN.}

    CD_SUCCESS("Created CuiAbsolute with canId %d and tr %f, and all local parameters set to 0.\n",canId,tr);
    return true;
}

// -----------------------------------------------------------------------------
bool roboticslab::CuiAbsolute::close()
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------
