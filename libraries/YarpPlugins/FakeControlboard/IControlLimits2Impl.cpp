// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <ColorDebug.hpp>

// ------------------- IControlLimits2 Related ------------------------------------

bool roboticslab::FakeControlboard::setVelLimits(int axis, double min, double max)
{
    CD_WARNING("Not implemented.\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getVelLimits(int axis, double *min, double *max)
{
    if (axis >= int(axes))
    {
        return false;
    }

    // yarpmotorgui's defaults (partitem.cpp)
    *min = 0;
    *max = 100;

    return true;
}

// -----------------------------------------------------------------------------
