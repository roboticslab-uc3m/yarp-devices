// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------- IControlMode related ------------------------------------

bool roboticslab::AmorControlboard::setPositionMode(int j)
{
    CD_DEBUG("(%d)\n", j);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setVelocityMode(int j)
{
    CD_DEBUG("(%d)\n", j);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setTorqueMode(int j)
{
    CD_DEBUG("(%d)\n", j);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setImpedancePositionMode(int j)
{
    CD_DEBUG("(%d)\n", j);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setImpedanceVelocityMode(int j)
{
    CD_DEBUG("(%d)\n", j);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setOpenLoopMode(int j)
{
    CD_DEBUG("(%d)\n", j);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getControlMode(int j, int *mode)
{
    //CD_DEBUG("(%d)\n", j);  //-- Way too verbose.
    if (!indexWithinRange(j))
        return false;
    *mode = VOCAB_CM_POSITION;
    return true;
}

// -----------------------------------------------------------------------------


bool roboticslab::AmorControlboard::getControlModes(int *modes)
{
    CD_DEBUG("\n");
    bool ok = true;
    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
        ok &= getControlMode(i, &(modes[i]));
    return ok;
}

// -----------------------------------------------------------------------------
