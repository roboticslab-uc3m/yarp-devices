// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <ColorDebug.h>

// ------------------- IControlMode Related ------------------------------------

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
