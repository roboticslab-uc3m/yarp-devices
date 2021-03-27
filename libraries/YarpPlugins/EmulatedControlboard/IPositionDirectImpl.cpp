// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlboard.hpp"

#include <yarp/os/Log.h>

// ------------------- IPositionDirect Related --------------------------------

bool roboticslab::EmulatedControlboard::setPosition(int j, double ref)
{
    if ((unsigned int)j > axes)
    {
        yError("Axis index exceeds number of axes");
        return false;
    }

    if (controlMode != POSITION_DIRECT_MODE)
    {
        yError("will not setPosition() as not in positionDirectMode");
        return false;
    }

    targetExposed[j] = ref;
    encRaw[j] = ref * encRawExposed[j];

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setPositions(const int n_joint, const int *joints, const double *refs)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setPosition(joints[i], refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setPositions(const double *refs)
{
    bool ok = true;

    for (int j = 0; j < axes; j++)
    {
        ok &= setPosition(j, refs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRefPosition(const int joint, double *ref)
{
    if ((unsigned int)joint > axes)
    {
        yError("Axis index exceeds number of axes");
        return false;
    }

    if (controlMode != POSITION_DIRECT_MODE)
    {
        yError("will not getRefPosition() as not in positionDirectMode");
        return false;
    }

    *ref = targetExposed[joint];

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRefPositions(double *refs)
{
    bool ok = true;

    for (int j = 0; j < axes; j++)
    {
        ok &= getRefPosition(j, &refs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRefPositions(const int n_joint, const int *joints, double *refs)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getRefPosition(joints[i], &refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
