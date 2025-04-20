// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

// ------------------- IControlLimits Related ------------------------------------

bool EmulatedControlBoard::setLimits(int axis, double min, double max)
{
    if (axis >= int(m_axes))
    {
        return false;
    }

    m_minLimits[axis] = min;
    m_maxLimits[axis] = max;

    yCDebug(ECB, "Range of axis %d set to: %f to %f", axis, min, max);

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getLimits(int axis, double *min, double *max)
{
    if (axis >= int(m_axes))
    {
        return false;
    }

    *min = m_minLimits[axis];
    *max = m_maxLimits[axis];

    yCDebug(ECB, "Range of axis %d read: %f to %f", axis, *min, *max);

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::setVelLimits(int axis, double min, double max)
{
    yCWarning(ECB, "setVelLimits() not implemented");
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getVelLimits(int axis, double *min, double *max)
{
    if (axis >= int(m_axes))
    {
        return false;
    }

    // yarpmotorgui's defaults (partitem.cpp)
    *min = -100;
    *max = 100;

    return true;
}

// -----------------------------------------------------------------------------
