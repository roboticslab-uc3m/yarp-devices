// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool LacqueyFetch::getAxes(int * ax)
{
    return getNumberOfMotorsRaw(ax);
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::getAxisNameRaw(int axis, std::string & name)
{
    yCITrace(LCQ, id(), "%d", axis);
    CHECK_JOINT(axis);
    name = axisName;
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type)
{
    yCITrace(LCQ, id(), "%d", axis);
    CHECK_JOINT(axis);
    type = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE;
    return true;
}

// -----------------------------------------------------------------------------
