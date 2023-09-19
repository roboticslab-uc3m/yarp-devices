// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------ IVelocity Related ----------------------------------------

bool EmulatedControlBoard::velocityMove(int j, double sp)  // velExposed = sp;
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    // Check if we are in velocity mode.
    if (controlMode != VELOCITY_MODE)
    {
        yCError(ECB, "will not velocityMove as not in velocityMode");
        return false;
    }

    velRaw[j] = sp * velRawExposed[j];
    jointStatus[j] = VELOCITY_MOVE;

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::velocityMove(const double *sp)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= velocityMove(i, sp[i]);
    }

    return ok;
}

// ----------------------------------------------------------------------------

bool EmulatedControlBoard::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    // must implement mask!
    return velocityMove(spds);
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getRefVelocity(const int joint, double *vel)
{
    yCWarning(ECB, "getRefVelocity() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getRefVelocities(double *vels)
{
    yCWarning(ECB, "getRefVelocities() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    yCWarning(ECB, "getRefVelocities() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------
