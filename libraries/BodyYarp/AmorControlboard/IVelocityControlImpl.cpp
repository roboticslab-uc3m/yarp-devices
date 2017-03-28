// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------ IVelocity Related ----------------------------------------

bool roboticslab::AmorControlboard::setVelocityMode()
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::velocityMove(int j, double sp)
{
    CD_DEBUG("(%d, %f)\n", j, sp);

    if (!indexWithinRange(j))
        return false;

    AMOR_VECTOR7 sps;

    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++) {
        if (i == j)
            sps[i] = sp;
        else
            sps[i] = 0;
    }

    return amor_set_velocities(handle, sps) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::velocityMove(const double *sp)
{
    CD_DEBUG("\n");
    bool ok = true;
    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
        ok &= velocityMove(i, sp[i]);
    return ok;
}

// ----------------------------------------------------------------------------
