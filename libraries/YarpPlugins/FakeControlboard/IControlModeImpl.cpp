// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <ColorDebug.hpp>

// ------------------- IControlLimits Related ------------------------------------

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

bool roboticslab::FakeControlboard::setImpedancePositionMode(int j)
{
    CD_DEBUG("(%d)\n", j);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setImpedanceVelocityMode(int j)
{
    CD_DEBUG("(%d)\n", j);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setOpenLoopMode(int j)
{
    CD_DEBUG("(%d)\n", j);
    return false;  // removed in YARP 2.3.70
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getControlMode(int j, int *mode)
{
    // CD_DEBUG("\n");  //-- Way too verbose.
    if (modePosVel == POSITION_MODE)
    {
        *mode = VOCAB_CM_POSITION;
    }
    else if (modePosVel == VELOCITY_MODE)
    {
        *mode = VOCAB_CM_VELOCITY;
    }
    else
    {
        CD_ERROR("Currently unsupported mode.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getControlModes(int *modes)
{
    CD_DEBUG("\n");
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= getControlMode(i, &modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
