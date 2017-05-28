// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------- IControlLimits2 related ------------------------------------

bool roboticslab::AmorControlboard::getAxisName(int axis, yarp::os::ConstString& name)
{
    CD_DEBUG("(%d)\n", axis);

    if (!indexWithinRange(axis))
    {
        return false;
    }

    switch (axis)
    {
        case 0:
            name = "A0";
            break;
        case 1:
            name = "A1";
            break;
        case 2:
            name = "A2";
            break;
        case 3:
            name = "A2.5";
            break;
        case 4:
            name = "A3";
            break;
        case 5:
            name = "A4";
            break;
        case 6:
            name = "A5";
            break;
        default:
            CD_ERROR("\n");
            return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    CD_DEBUG("(%d)\n", axis);

    if (!indexWithinRange(axis))
    {
        return false;
    }

    type = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE;

    return true;
}

// -----------------------------------------------------------------------------
