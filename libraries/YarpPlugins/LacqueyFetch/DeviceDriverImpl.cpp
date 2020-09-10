// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool LacqueyFetch::open(yarp::os::Searchable& config)
{
    CD_DEBUG("%s\n", config.toString().c_str());

    canId = config.check("canId", yarp::os::Value(0), "can bus ID").asInt8();
    CD_SUCCESS("Created LacqueyFetch with canId %d.\n", canId);
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::close()
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------
