// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- IPositionDirect Related --------------------------------

bool EmulatedControlboard::setPosition(int j, double ref)
{
    if ((unsigned int)j > axes)
    {
        yCError(ECB, "Axis index exceeds number of axes");
        return false;
    }

    if (controlMode != POSITION_DIRECT_MODE)
    {
        yCError(ECB, "will not setPosition() as not in positionDirectMode");
        return false;
    }

    targetExposed[j] = ref;
    encRaw[j] = ref * encRawExposed[j];

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::setPositions(const int n_joint, const int *joints, const double *refs)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setPosition(joints[i], refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::setPositions(const double *refs)
{
    bool ok = true;

    for (int j = 0; j < axes; j++)
    {
        ok &= setPosition(j, refs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::getRefPosition(const int joint, double *ref)
{
    if ((unsigned int)joint > axes)
    {
        yCError(ECB, "Axis index exceeds number of axes");
        return false;
    }

    if (controlMode != POSITION_DIRECT_MODE)
    {
        yCError(ECB, "will not getRefPosition() as not in positionDirectMode");
        return false;
    }

    *ref = targetExposed[joint];

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::getRefPositions(double *refs)
{
    bool ok = true;

    for (int j = 0; j < axes; j++)
    {
        ok &= getRefPosition(j, &refs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlboard::getRefPositions(const int n_joint, const int *joints, double *refs)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getRefPosition(joints[i], &refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
