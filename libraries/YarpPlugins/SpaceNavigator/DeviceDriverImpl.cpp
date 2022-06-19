// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

using namespace roboticslab;

constexpr auto DEFAULT_DEADBAND = 0.125;

namespace
{
    YARP_LOG_COMPONENT(SPNAV, "rl.SpaceNavigator")
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::open(yarp::os::Searchable & config)
{
    deadband = config.check("deadband", yarp::os::Value(DEFAULT_DEADBAND), "deadband [0,1]").asFloat64();

    if (spnav_open() == -1)
    {
        yCError(SPNAV) << "Failed to connect to the space navigator daemon";
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
