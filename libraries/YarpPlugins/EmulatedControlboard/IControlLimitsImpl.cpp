// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlboard.hpp"

#include <ColorDebug.h>

// ------------------- IControlLimits Related ------------------------------------

bool roboticslab::EmulatedControlboard::setLimits(int axis, double min, double max)
{
    if (axis >= int(axes))
    {
        return false;
    }

    minLimit[axis] = min;
    maxLimit[axis] = max;

    CD_DEBUG("Range of axis %d set to: %f to %f\n", axis, min, max);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getLimits(int axis, double *min, double *max)
{
    if (axis >= int(axes))
    {
        return false;
    }

    *min = minLimit[axis];
    *max = maxLimit[axis];
    
    CD_DEBUG("Range of axis %d read: %f to %f.\n", axis, *min, *max);
    
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setVelLimits(int axis, double min, double max)
{
    CD_WARNING("Not implemented.\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getVelLimits(int axis, double *min, double *max)
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
