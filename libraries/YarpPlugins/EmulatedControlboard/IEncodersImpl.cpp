// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlboard.hpp"

using namespace roboticslab;

// ------------------ IEncoders Related -----------------------------------------

bool EmulatedControlboard::resetEncoder(int j)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    return setEncoder(j, 0.0);
  }

// -----------------------------------------------------------------------------

bool EmulatedControlboard::resetEncoders()
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= resetEncoder(i);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::setEncoder(int j, double val)  // encExposed = val;
{
    setEncRaw(j, val * encRawExposed[j]);
    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::setEncoders(const double *vals)
{
    std::vector<double> v(axes);

    for (unsigned int i = 0; i < axes; i++)
    {
        v[i] = vals[i] * encRawExposed[i];
    }

    setEncsRaw(v);
    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::getEncoder(int j, double *v)
{
    *v = getEncExposed(j);
    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::getEncoders(double *encs)
{
    std::vector<double> v = getEncsExposed();

    for (unsigned int i = 0; i < axes; i++)
    {
        encs[i] = v[i];
    }

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::getEncoderSpeed(int j, double *sp)
{
    // Make it easy, give the current reference speed.
    *sp = velRaw[j] / velRawExposed[j];  // begins to look like we should use semaphores.
    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::getEncoderSpeeds(double *spds)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= getEncoderSpeed(i, &spds[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::getEncoderAcceleration(int j, double *spds)
{
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::getEncoderAccelerations(double *accs)
{
    return false;
}

// ------------------ IEncodersTimed Related -----------------------------------------

bool EmulatedControlboard::getEncodersTimed(double *encs, double *time)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= getEncoderTimed(i, &encs[i], &time[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::getEncoderTimed(int j, double *encs, double *time)
{
    getEncoder(j, encs);
    *time = yarp::os::Time::now();

    return true;
}

// -----------------------------------------------------------------------------
