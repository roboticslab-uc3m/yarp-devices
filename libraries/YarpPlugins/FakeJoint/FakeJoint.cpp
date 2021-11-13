// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(FAKE, "rl.FakeJoint")
}

// -----------------------------------------------------------------------------

bool FakeJoint::open(yarp::os::Searchable & config)
{
#if YARP_VERSION_MINOR < 6
    yCDebug(FAKE) << "Config:" << config.toString();
#endif

    jointName = config.check("jointName", yarp::os::Value(""), "name of the fake joint").asString();
    return true;
}

// -----------------------------------------------------------------------------
