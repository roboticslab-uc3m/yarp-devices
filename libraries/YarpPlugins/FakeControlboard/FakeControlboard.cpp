// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <ColorDebug.hpp>

// ------------------- Miscellanea ------------------------------------

bool roboticslab::FakeControlboard::setPositionMode(int j)
{
    CD_DEBUG("(%d)\n", j);

    if (modePosVel == POSITION_MODE)
    {
        return true;  // Simply return true if we were already in pos mode.
    }

    // Do anything additional before setting flag to pos...
    if (!stop(j))
    {
        CD_ERROR("failed to stop joint %d\n", j);
        return false;
    }

    modePosVel = POSITION_MODE;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setVelocityMode(int j)
{
    CD_DEBUG("(%d)\n", j);
    modePosVel = VELOCITY_MODE;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setTorqueMode(int j)
{
    CD_DEBUG("(%d)\n", j);
    return true;
}

// -----------------------------------------------------------------------------
