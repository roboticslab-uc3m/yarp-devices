// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#include <yarp/os/Vocab.h>

// ------------------- IControlMode related ------------------------------------

bool roboticslab::AmorControlboard::getControlMode(int j, int *mode)
{
    //CD_DEBUG("(%d)\n", j);  //-- Way too verbose.
    if (!indexWithinRange(j))
    {
        return false;
    }

    *mode = controlMode;
    return true;
}

// -----------------------------------------------------------------------------


bool roboticslab::AmorControlboard::getControlModes(int *modes)
{
    //CD_DEBUG("\n");  //-- Way too verbose.
    bool ok = true;

    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        ok &= getControlMode(i, &modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    bool ok = true;

    for (unsigned int i = 0; i < n_joint; i++)
    {
        ok &= getControlMode(joints[i], &modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setControlMode(const int j, const int mode)
{
    CD_DEBUG("(%d, %s)\n", j, yarp::os::Vocab::decode(mode).c_str());

    if (!indexWithinRange(j))
    {
        return false;
    }

    controlMode = mode;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    bool ok = true;

    for (unsigned int i = 0; i < n_joint; i++)
    {
        ok &= setControlMode(joints[i], modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setControlModes(int *modes)
{
    CD_DEBUG("\n");

    bool ok = true;

    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        ok &= setControlMode(i, modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
