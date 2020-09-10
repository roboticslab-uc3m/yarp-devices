// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getAxisNameRaw(int axis, std::string & name)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);
    name = vars.axisName;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);
    type = static_cast<yarp::dev::JointTypeEnum>(vars.jointType);
    return true;
}

// -----------------------------------------------------------------------------
