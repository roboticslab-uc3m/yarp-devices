// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#include <yarp/os/Log.h>

// ------------------ IEncoders related -----------------------------------------

bool roboticslab::AmorControlboard::resetEncoder(int j)
{
    yError("resetEncoder() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::resetEncoders()
{
    yError("resetEncoders() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setEncoder(int j, double val)
{
    yError("setEncoder() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setEncoders(const double *vals)
{
    yError("setEncoders() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoder(int j, double *v)
{
    yTrace("%d", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yError("amor_get_actual_positions() failed: %s", amor_error());
        return false;
    }

    *v = toDeg(positions[j]);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoders(double *encs)
{
    yTrace("");

    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yError("amor_get_actual_positions() failed: %s", amor_error());
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
    yTrace("%d", j);

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

    *sp = toDeg(velocities[j]);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoderSpeeds(double *spds)
{
    yTrace("");

    AMOR_VECTOR7 velocities;

    if (amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        yError("amor_get_actual_velocities() failed: %s", amor_error());
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
    //yError("getEncoderAcceleration() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoderAccelerations(double *accs)
{
    //yError("getEncoderAccelerations() not available");
    return false;
}

// ------------------ IEncodersTimed related -----------------------------------------

bool roboticslab::AmorControlboard::getEncodersTimed(double *encs, double *time)
{
    yTrace("");
    double now = yarp::os::Time::now();
    std::fill_n(time, AMOR_NUM_JOINTS, now);
    return getEncoders(encs);
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoderTimed(int j, double *encs, double *time)
{
    yTrace("%d", j);
    *time = yarp::os::Time::now();
    return getEncoder(j, encs);
}

// -----------------------------------------------------------------------------
