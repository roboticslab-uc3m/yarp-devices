// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(FAKE, "yarp.device.fakejoint")
}

// -----------------------------------------------------------------------------

bool FakeJoint::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(FAKE) << "Failed to parse parameters";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
