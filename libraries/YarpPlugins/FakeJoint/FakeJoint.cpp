// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool FakeJoint::open(yarp::os::Searchable & config)
{
    jointName = config.check("jointName", yarp::os::Value(""), "name of the fake joint").asString();
    return true;
}

// -----------------------------------------------------------------------------
