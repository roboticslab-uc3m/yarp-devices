// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlboard.hpp"

#include <ColorDebug.h>

// ------------------- Miscellanea ------------------------------------

bool roboticslab::EmulatedControlboard::setPositionMode(int j)
{
    CD_DEBUG("(%d)\n", j);

    if (controlMode == POSITION_MODE)
    {
        return true;  // Simply return true if we were already in pos mode.
    }

    // Do anything additional before setting flag to pos...
    if (!stop(j))
    {
        CD_ERROR("failed to stop joint %d\n", j);
        return false;
    }

    controlMode = POSITION_MODE;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setVelocityMode(int j)
{
    CD_DEBUG("(%d)\n", j);
    controlMode = VELOCITY_MODE;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setTorqueMode(int j)
{
    CD_DEBUG("(%d)\n", j);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setPositionDirectMode(int j)
{
    CD_DEBUG("(%d)\n", j);

    if (controlMode == POSITION_DIRECT_MODE)
    {
        return true;  // Simply return true if we were already in posd mode.
    }

    // Do anything additional before setting flag to posd...
    if (!stop(j))
    {
        CD_ERROR("failed to stop joint %d\n", j);
        return false;
    }

    controlMode = POSITION_DIRECT_MODE;
    return true;
}

// -----------------------------------------------------------------------------
