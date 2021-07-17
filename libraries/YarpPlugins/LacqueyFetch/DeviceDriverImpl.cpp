// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool LacqueyFetch::open(yarp::os::Searchable& config)
{
    yCDebug(LCQ) << "Config:" << config.toString();

    canId = config.check("canId", yarp::os::Value(0), "can bus ID").asInt8();
    yCInfo(LCQ) << "Created LacqueyFetch with canId" << canId;
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::close()
{
    return true;
}

// -----------------------------------------------------------------------------
