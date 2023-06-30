// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ForceTorqueCan.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool ForceTorqueCan::open(yarp::os::Searchable & config)
{
    return true;
}

// -----------------------------------------------------------------------------

bool ForceTorqueCan::close()
{
    return true;
}

// -----------------------------------------------------------------------------
