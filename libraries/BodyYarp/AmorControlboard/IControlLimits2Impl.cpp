// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------- IControlLimits2 related ------------------------------------

bool roboticslab::AmorControlboard::setVelLimits(int axis, double min, double max)
{
    CD_ERROR("Not available (%d, %f, %f).\n", axis, min, max);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getVelLimits(int axis, double *min, double *max)
{
    CD_DEBUG("(%d)\n", axis);

    if (!indexWithinRange(axis))
    {
        return false;
    }

    AMOR_JOINT_INFO parameters;

    if (amor_get_joint_info(handle, axis, &parameters) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    *min = 0.0;
    *max = toDeg(parameters.maxVelocity);

    return true;
}

// -----------------------------------------------------------------------------
