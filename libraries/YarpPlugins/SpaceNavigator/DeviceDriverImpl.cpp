// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

bool SpaceNavigator::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(SPNAV) << "Failed to parse parameters";
        return false;
    }

    if (m_deadband < 0.0 || m_deadband > 1.0)
    {
        yCError(SPNAV) << "Invalid deadband value (must be in [0,1]):" << m_deadband;
        return false;
    }

    if (::spnav_open() == -1)
    {
        yCError(SPNAV) << "Failed to connect to the space navigator daemon";
        return false;
    }

    return yarp::os::Thread::start();
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::close()
{
    return yarp::os::Thread::stop() & ::spnav_close() != -1;
}

// -----------------------------------------------------------------------------
