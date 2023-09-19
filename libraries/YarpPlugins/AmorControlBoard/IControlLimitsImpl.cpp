// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- IControlLimits related ------------------------------------

bool AmorControlBoard::setLimits(int axis, double min, double max)
{
    yCError(AMOR, "setLimits() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getLimits(int axis, double *min, double *max)
{
    yCTrace(AMOR, "%d", axis);

    if (!indexWithinRange(axis))
    {
        return false;
    }

    AMOR_JOINT_INFO parameters;

    if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, axis, &parameters) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_joint_info() failed: %s", amor_error());
        return false;
    }

    if (parameters.lowerJointLimit == 0.0 && parameters.upperJointLimit == 0.0)
    {
        *min = -180.0;
        *max = 180.0;
    }
    else
    {
        *min = toDeg(parameters.lowerJointLimit);
        *max = toDeg(parameters.upperJointLimit);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::setVelLimits(int axis, double min, double max)
{
    yCError(AMOR, "setVelLimits() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getVelLimits(int axis, double *min, double *max)
{
    yCTrace(AMOR, "%d", axis);

    if (!indexWithinRange(axis))
    {
        return false;
    }

    AMOR_JOINT_INFO parameters;

    if (std::lock_guard lock(handleMutex); amor_get_joint_info(handle, axis, &parameters) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_joint_info() failed: %s", amor_error());
        return false;
    }

    *max = toDeg(parameters.maxVelocity);
    *min = -(*max);

    return true;
}

// -----------------------------------------------------------------------------
