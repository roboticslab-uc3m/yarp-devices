// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <yarp/os/Property.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool LacqueyFetch::open(yarp::os::Searchable & config)
{
    canId = config.check("canId", yarp::os::Value(0), "can bus ID").asInt8();
    axisName = config.check("name", yarp::os::Value(""), "axis name").asString();
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::close()
{
    return true;
}

// -----------------------------------------------------------------------------
