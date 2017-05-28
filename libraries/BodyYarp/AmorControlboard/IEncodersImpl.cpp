// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------ IEncoders related -----------------------------------------

bool roboticslab::AmorControlboard::resetEncoder(int j)
{
    CD_ERROR("Not available (%d).\n", j);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::resetEncoders()
{
    CD_ERROR("Not available.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setEncoder(int j, double val)
{
    CD_ERROR("Not available (%d, %f).\n", j, val);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setEncoders(const double *vals)
{
    CD_ERROR("Not available.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoder(int j, double *v)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("Could not retrieve current positions.\n");
        return false;
    }

    *v = toDeg(positions[j]);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoders(double *encs)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("Could not retrieve current positions.\n");
        return false;
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        encs[j] = toDeg(positions[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoderSpeed(int j, double *sp)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    if (amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("Could not retrieve current velocities.\n");
        return false;
    }

    *sp = toDeg(velocities[j]);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoderSpeeds(double *spds)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 velocities;

    if (amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("Could not retrieve current velocities.\n");
        return false;
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        spds[j] = toDeg(velocities[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoderAcceleration(int j, double *spds)
{
    CD_ERROR("Not available.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoderAccelerations(double *accs)
{
    CD_ERROR("Not available.\n");
    return false;
}

// -----------------------------------------------------------------------------
