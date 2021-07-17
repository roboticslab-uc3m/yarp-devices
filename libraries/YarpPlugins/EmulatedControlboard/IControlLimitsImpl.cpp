// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- IControlLimits Related ------------------------------------

bool EmulatedControlboard::setLimits(int axis, double min, double max)
{
    if (axis >= int(axes))
    {
        return false;
    }

    minLimit[axis] = min;
    maxLimit[axis] = max;

    yCDebug(ECB, "Range of axis %d set to: %f to %f", axis, min, max);

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::getLimits(int axis, double *min, double *max)
{
    if (axis >= int(axes))
    {
        return false;
    }

    *min = minLimit[axis];
    *max = maxLimit[axis];

    yCDebug(ECB, "Range of axis %d read: %f to %f", axis, *min, *max);

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::setVelLimits(int axis, double min, double max)
{
    yCWarning(ECB, "setVelLimits() not implemented");
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::getVelLimits(int axis, double *min, double *max)
{
    if (axis >= int(axes))
    {
        return false;
    }

    // yarpmotorgui's defaults (partitem.cpp)
    *min = -100;
    *max = 100;

    return true;
}

// -----------------------------------------------------------------------------
