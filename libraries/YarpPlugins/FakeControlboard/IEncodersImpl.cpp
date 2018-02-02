// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

// ------------------ IEncoders Related -----------------------------------------

bool roboticslab::FakeControlboard::resetEncoder(int j)
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    return setEncoder(j, 0.0);
  }

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::resetEncoders()
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= resetEncoder(i);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setEncoder(int j, double val)  // encExposed = val;
{
    setEncRaw(j, val * encRawExposed[j]);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setEncoders(const double *vals)
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

bool roboticslab::FakeControlboard::getEncoder(int j, double *v)
{
    *v = getEncExposed(j);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getEncoders(double *encs)
{
    std::vector<double> v = getEncsExposed();

    for (unsigned int i = 0; i < axes; i++)
    {
        encs[i] = v[i];
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getEncoderSpeed(int j, double *sp)
{
    // Make it easy, give the current reference speed.
    *sp = velRaw[j] / velRawExposed[j];  // begins to look like we should use semaphores.
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getEncoderSpeeds(double *spds)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= getEncoderSpeed(i, &spds[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getEncoderAcceleration(int j, double *spds)
{
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getEncoderAccelerations(double *accs)
{
    return false;
}

// -----------------------------------------------------------------------------
