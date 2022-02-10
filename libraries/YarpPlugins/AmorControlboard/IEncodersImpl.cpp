// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------ IEncoders related -----------------------------------------

bool AmorControlboard::resetEncoder(int j)
{
    yCError(AMOR, "resetEncoder() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::resetEncoders()
{
    yCError(AMOR, "resetEncoders() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::setEncoder(int j, double val)
{
    yCError(AMOR, "setEncoder() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::setEncoders(const double *vals)
{
    yCError(AMOR, "setEncoders() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::getEncoder(int j, double *v)
{
    yCTrace(AMOR, "%d", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (std::lock_guard<std::mutex> lock(handleMutex); amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_positions() failed: %s", amor_error());
        return false;
    }

    *v = toDeg(positions[j]);

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::getEncoders(double *encs)
{
    yCTrace(AMOR, "");

    AMOR_VECTOR7 positions;

    if (std::lock_guard<std::mutex> lock(handleMutex); amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_positions() failed: %s", amor_error());
        return false;
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        encs[j] = toDeg(positions[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::getEncoderSpeed(int j, double *sp)
{
    yCTrace(AMOR, "%d", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    if (std::lock_guard<std::mutex> lock(handleMutex); amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_velocities() failed: %s", amor_error());
        return false;
    }

    *sp = toDeg(velocities[j]);

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::getEncoderSpeeds(double *spds)
{
    yCTrace(AMOR, "");

    AMOR_VECTOR7 velocities;

    if (std::lock_guard<std::mutex> lock(handleMutex); amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_velocities() failed: %s", amor_error());
        return false;
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        spds[j] = toDeg(velocities[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::getEncoderAcceleration(int j, double *spds)
{
    //yCError(AMOR, "getEncoderAcceleration() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::getEncoderAccelerations(double *accs)
{
    //yCError(AMOR, "getEncoderAccelerations() not available");
    return false;
}

// ------------------ IEncodersTimed related -----------------------------------------

bool AmorControlboard::getEncodersTimed(double *encs, double *time)
{
    yCTrace(AMOR, "");
    double now = yarp::os::Time::now();
    std::fill_n(time, AMOR_NUM_JOINTS, now);
    return getEncoders(encs);
}

// -----------------------------------------------------------------------------

bool AmorControlboard::getEncoderTimed(int j, double *encs, double *time)
{
    yCTrace(AMOR, "%d", j);
    *time = yarp::os::Time::now();
    return getEncoder(j, encs);
}

// -----------------------------------------------------------------------------
