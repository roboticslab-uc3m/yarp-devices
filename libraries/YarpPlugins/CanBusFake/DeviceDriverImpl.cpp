// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusFake.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CanBusFake::open(yarp::os::Searchable& config)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusFake::close()
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

