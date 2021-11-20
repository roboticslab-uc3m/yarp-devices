// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

#include <yarp/conf/version.h>

using namespace roboticslab;

#if !defined(YARP_VERSION_COMPARE) // < 3.6.0
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

namespace
{
    YARP_LOG_COMPONENT(FAKE, "rl.FakeJoint")
}
#endif

// -----------------------------------------------------------------------------

bool FakeJoint::open(yarp::os::Searchable & config)
{
#if !defined(YARP_VERSION_COMPARE) // < 3.6.0
    yCDebug(FAKE) << "Config:" << config.toString();
#endif

    jointName = config.check("jointName", yarp::os::Value(""), "name of the fake joint").asString();
    return true;
}

// -----------------------------------------------------------------------------
