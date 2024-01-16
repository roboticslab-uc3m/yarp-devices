// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool FakeJoint::open(yarp::os::Searchable & config)
{
    axisName = config.check("axisName", yarp::os::Value(""), "name of the fake axis").asString();
    jointType = config.check("jointType", yarp::os::Value(yarp::dev::VOCAB_JOINTTYPE_UNKNOWN), "type of the fake joint [atrv|atpr|unkn]").asVocab32();
    return true;
}

// -----------------------------------------------------------------------------
