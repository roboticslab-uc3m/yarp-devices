// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- IControlMode related ------------------------------------

bool AmorControlboard::getControlMode(int j, int *mode)
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


bool AmorControlboard::getControlModes(int *modes)
{
    bool ok = true;

    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        ok &= getControlMode(i, &modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::getControlModes(const int n_joint, const int *joints, int *modes)
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

bool AmorControlboard::setControlMode(const int j, const int mode)
{
#if YARP_VERSION_MINOR >= 5
    yCTrace(AMOR, "%d %s", j, yarp::os::Vocab32::decode(mode).c_str());
#else
    yCTrace(AMOR, "%d %s", j, yarp::os::Vocab::decode(mode).c_str());
#endif

    if (!indexWithinRange(j))
    {
        return false;
    }

    controlMode = mode;

    return true;
}

// -----------------------------------------------------------------------------

bool AmorControlboard::setControlModes(const int n_joint, const int *joints, int *modes)
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

bool AmorControlboard::setControlModes(int *modes)
{
    bool ok = true;

    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
    {
        ok &= setControlMode(i, modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
