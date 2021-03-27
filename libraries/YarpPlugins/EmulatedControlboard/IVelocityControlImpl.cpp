// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlboard.hpp"

#include <yarp/os/Log.h>

// ------------------ IVelocity Related ----------------------------------------

bool roboticslab::EmulatedControlboard::velocityMove(int j, double sp)  // velExposed = sp;
{
    if ((unsigned int)j > axes)
    {
        return false;
    }

    // Check if we are in velocity mode.
    if (controlMode != VELOCITY_MODE)
    {
        yError("EmulatedControlboard will not velocityMove as not in velocityMode");
        return false;
    }

    velRaw[j] = sp * velRawExposed[j];
    jointStatus[j] = VELOCITY_MOVE;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::velocityMove(const double *sp)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= velocityMove(i, sp[i]);
    }

    return ok;
}

// ----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    // must implement mask!
    return velocityMove(spds);
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRefVelocity(const int joint, double *vel)
{
    yWarning("getRefVelocity() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRefVelocities(double *vels)
{
    yWarning("getRefVelocities() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    yWarning("getRefVelocities() not implemented yet");
    return false;
}

// -----------------------------------------------------------------------------
