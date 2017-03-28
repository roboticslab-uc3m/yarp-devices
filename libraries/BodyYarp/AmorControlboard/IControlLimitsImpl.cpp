// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------- IControlLimits Related ------------------------------------

bool roboticslab::AmorControlboard::setLimits(int axis, double min, double max)
{
    CD_DEBUG("(%d, %f, %f)\n", axis, min, max);
    if (!indexWithinRange(axis))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getLimits(int axis, double *min, double *max)
{
    CD_DEBUG("(%d)\n", axis);
    if (!indexWithinRange(axis))
        return false;
    return true;
}

// -----------------------------------------------------------------------------
