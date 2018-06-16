// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <ColorDebug.h>

// ------------------- IControlLimits Related ------------------------------------

bool roboticslab::FakeControlboard::setLimits(int axis, double min, double max)
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

bool roboticslab::FakeControlboard::getLimits(int axis, double *min, double *max)
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
