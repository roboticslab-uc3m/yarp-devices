// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool SpaceNavigator::open(yarp::os::Searchable & config)
{
    yDebug() << "SpaceNavigator config:" << config.toString();

    deadband = config.check("deadband", yarp::os::Value(DEFAULT_DEADBAND), "deadband [0,1]").asFloat64();

    if (spnav_open() == -1)
    {
        yError() << "Failed to connect to the space navigator daemon";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::close()
{
    return spnav_close() != -1;
}

// -----------------------------------------------------------------------------
