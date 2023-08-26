// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlBoard.hpp"

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- IControlMode related ------------------------------------

bool AmorControlBoard::getControlMode(int j, int *mode)
{
    yCTrace(AMOR, "%d", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    *mode = controlMode;
    return true;
}

// -----------------------------------------------------------------------------


bool AmorControlBoard::getControlModes(int *modes)
{
    bool ok = true;

    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        ok &= getControlMode(i, &modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::getControlModes(const int n_joint, const int *joints, int *modes)
{
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

bool AmorControlBoard::setControlMode(const int j, const int mode)
{
    yCTrace(AMOR, "%d %s", j, yarp::os::Vocab32::decode(mode).c_str());

    if (!indexWithinRange(j))
    {
        return false;
    }

    controlMode = mode;

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlBoard::setControlModes(const int n_joint, const int *joints, int *modes)
{
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

bool AmorControlBoard::setControlModes(int *modes)
{
    bool ok = true;

    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        ok &= setControlMode(i, modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
