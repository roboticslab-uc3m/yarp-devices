// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <ColorDebug.h>

// ------------------- IAxisInfo Related ------------------------------------

bool roboticslab::DextraControlboardUSB::getAxisName(int axis, std::string &name)
{
    CD_DEBUG("axis\n");
    CHECK_JOINT(axis);

    switch (axis)
    {
    case 0:
        name = "abductor";
        break;
    case 1:
        name = "thumb";
        break;
    case 2:
        name = "index";
        break;
    case 3:
        name = "middle";
        break;
    case 4:
        name = "ring";
        break;
    case 5:
        name = "pinky";
        break;
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getJointType(int axis, yarp::dev::JointTypeEnum &type)
{
    CD_DEBUG("axis\n");
    CHECK_JOINT(axis);
    type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;
    return true;
}

// -----------------------------------------------------------------------------
