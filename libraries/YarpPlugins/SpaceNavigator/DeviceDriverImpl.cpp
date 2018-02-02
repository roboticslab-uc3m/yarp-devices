// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::SpaceNavigator::open(yarp::os::Searchable& config)
{
    if (spnav_open() == -1)
    {
        CD_ERROR("Failed to connect to the space navigator daemon\n");
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::SpaceNavigator::close()
{
    return spnav_close() != -1;
}

// -----------------------------------------------------------------------------
