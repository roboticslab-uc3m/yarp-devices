// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <yarp/os/LogStream.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool LacqueyFetch::open(yarp::os::Searchable& config)
{
    yDebug() << "LacqueyFetch config:" << config.toString();

    canId = config.check("canId", yarp::os::Value(0), "can bus ID").asInt8();
    yInfo() << "Created LacqueyFetch with canId" << canId;
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::close()
{
    return true;
}

// -----------------------------------------------------------------------------
