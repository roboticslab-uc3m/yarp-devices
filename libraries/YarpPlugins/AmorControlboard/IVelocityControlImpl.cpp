// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#include <yarp/os/Log.h>

// ------------------ IVelocityControl related ----------------------------------------

bool roboticslab::AmorControlboard::velocityMove(int j, double sp)
{
    yTrace("%d %f", j, sp);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    if (amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yError("amor_get_actual_velocities() failed: %s", amor_error());
        return false;
    }

    velocities[j] = toRad(sp);

    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::velocityMove(const double *sp)
{
    AMOR_VECTOR7 velocities;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        velocities[j] = toRad(sp[j]);
    }

    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS;
}

// ----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    yTrace("%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    if (n_joint < AMOR_NUM_JOINTS && amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yError("amor_get_actual_velocities() failed: %s", amor_error());
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        velocities[joints[j]] = toRad(spds[j]);
    }

    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefVelocity(const int joint, double *vel)
{
    yTrace("%d", joint);

    if (!indexWithinRange(joint))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    if (amor_get_req_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yError("amor_get_req_velocities() failed: %s", amor_error());
        return false;
    }

    *vel = toDeg(velocities[joint]);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefVelocities(double *vels)
{
    yTrace("");

    AMOR_VECTOR7 velocities;

    if (amor_get_req_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yError("amor_get_req_velocities() failed: %s", amor_error());
        return false;
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        vels[j] = toDeg(velocities[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    yTrace("%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    if (amor_get_req_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yError("amor_get_req_velocities() failed: %s", amor_error());
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        vels[j] = toDeg(velocities[joints[j]]);
    }

    return true;
}

// -----------------------------------------------------------------------------
