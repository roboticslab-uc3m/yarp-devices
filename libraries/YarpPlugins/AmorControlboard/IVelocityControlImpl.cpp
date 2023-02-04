// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------ IVelocityControl related ----------------------------------------

bool AmorControlboard::velocityMove(int j, double sp)
{
    yCTrace(AMOR, "%d %f", j, sp);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    if (std::lock_guard lock(handleMutex); amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_velocities() failed: %s", amor_error());
        return false;
    }

    velocities[j] = toRad(sp);

    std::lock_guard lock(handleMutex);
    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::velocityMove(const double *sp)
{
    AMOR_VECTOR7 velocities;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        velocities[j] = toRad(sp[j]);
    }

    std::lock_guard lock(handleMutex);
    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS;
}

// ----------------------------------------------------------------------------

bool AmorControlboard::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    if (std::lock_guard lock(handleMutex); n_joint < AMOR_NUM_JOINTS && amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_velocities() failed: %s", amor_error());
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        velocities[joints[j]] = toRad(spds[j]);
    }

    std::lock_guard lock(handleMutex);
    return amor_set_velocities(handle, velocities) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::getRefVelocity(const int joint, double *vel)
{
    yCTrace(AMOR, "%d", joint);

    if (!indexWithinRange(joint))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    if (std::lock_guard lock(handleMutex); amor_get_req_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_req_velocities() failed: %s", amor_error());
        return false;
    }

    *vel = toDeg(velocities[joint]);

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::getRefVelocities(double *vels)
{
    yCTrace(AMOR, "");

    AMOR_VECTOR7 velocities;

    if (std::lock_guard lock(handleMutex); amor_get_req_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_req_velocities() failed: %s", amor_error());
        return false;
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        vels[j] = toDeg(velocities[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    if (std::lock_guard lock(handleMutex); amor_get_req_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_req_velocities() failed: %s", amor_error());
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        vels[j] = toDeg(velocities[joints[j]]);
    }

    return true;
}

// -----------------------------------------------------------------------------
