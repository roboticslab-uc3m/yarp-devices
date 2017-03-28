// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------ IEncoders Related -----------------------------------------

bool roboticslab::AmorControlboard::resetEncoder(int j)
{
    CD_DEBUG("(%d)\n", j);
    if (!indexWithinRange(j))
        return false;
    return setEncoder(j, 0.0);
  }

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::resetEncoders()
{
    CD_DEBUG("\n");
    bool ok = true;
    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
        ok &= resetEncoder(i);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setEncoder(int j, double val)
{
    CD_DEBUG("(%d, %f)\n", j, val);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setEncoders(const double *vals)
{
    CD_DEBUG("\n");
    bool ok = true;
    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
        ok &= setEncoder(i, vals[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoder(int j, double *v)
{
    //CD_DEBUG("(%d)\n", j);  //-- Way too verbose
    if (!indexWithinRange(j))
        return false;
    *v = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoders(double *encs)
{
    CD_DEBUG("\n");
    bool ok = true;
    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
        ok &= getEncoder(i, &encs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoderSpeed(int j, double *sp)
{
    //CD_DEBUG("(%d)\n", j);  //-- Way too verbose
    if (!indexWithinRange(j))
        return false;
    // Make it easy, give the current reference speed.
    *sp = 0;  // begins to look like we should use semaphores.
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoderSpeeds(double *spds)
{
    CD_DEBUG("\n");
    bool ok = true;
    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
        ok &= getEncoderSpeed(i, &spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoderAcceleration(int j, double *spds)
{
    //CD_DEBUG("(%d)\n", j);  //-- Way too verbose
    if (!indexWithinRange(j))
        return false;
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoderAccelerations(double *accs)
{
    CD_DEBUG("\n");
    return false;
}

// -----------------------------------------------------------------------------
