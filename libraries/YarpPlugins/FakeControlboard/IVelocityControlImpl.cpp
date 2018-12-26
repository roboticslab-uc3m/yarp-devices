// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <ColorDebug.h>

// ------------------ IVelocity Related ----------------------------------------

bool roboticslab::FakeControlboard::velocityMove(int j, double sp)  // velExposed = sp;
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    // Check if we are in velocity mode.
    if (controlMode != VELOCITY_MODE)
    {
        CD_ERROR("FakeControlboard will not velocityMove as not in velocityMode\n");
        return false;
    }

    velRaw[j] = sp * velRawExposed[j];
    jointStatus[j] = VELOCITY_MOVE;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::velocityMove(const double *sp)
{
    CD_DEBUG("Vel:");

    for (unsigned int i = 0; i < axes; i++)
    {
        CD_DEBUG_NO_HEADER(" %+.6f", velRaw[i]);
    }

    CD_DEBUG_NO_HEADER("\n");
    
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= velocityMove(i, sp[i]);
    }

    return ok;
}

// ----------------------------------------------------------------------------
