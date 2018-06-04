// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------ IVelocityControl related ----------------------------------------

bool roboticslab::AmorControlboard::velocityMove(int j, double sp)
{
    CD_DEBUG("(%d, %f)\n", j, sp);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    if (amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    velocities[j] = toRad(sp);

    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::velocityMove(const double *sp)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 velocities;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        velocities[j] = toRad(sp[j]);
    }

    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS;
}

// ----------------------------------------------------------------------------
