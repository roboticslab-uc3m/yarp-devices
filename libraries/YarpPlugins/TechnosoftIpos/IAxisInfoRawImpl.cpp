// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getAxisNameRaw(int axis, std::string & name)
{
    yCITrace(IPOS, id(), "%d", axis);
    CHECK_JOINT(axis);
    name = params.m_name;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type)
{
    yCITrace(IPOS, id(), "%d", axis);
    CHECK_JOINT(axis);
    type = static_cast<yarp::dev::JointTypeEnum>(jointType);
    return true;
}

// -----------------------------------------------------------------------------
