// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

// ------------------- IPositionDirect Related --------------------------------

bool EmulatedControlBoard::setPosition(int j, double ref)
{
    if ((unsigned int)j > m_axes)
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
    encRaw[j] = ref * m_encRawExposeds[j];

    return true;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::setPositions(const int n_joint, const int *joints, const double *refs)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setPosition(joints[i], refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::setPositions(const double *refs)
{
    bool ok = true;

    for (int j = 0; j < m_axes; j++)
    {
        ok &= setPosition(j, refs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getRefPosition(const int joint, double *ref)
{
    if ((unsigned int)joint > m_axes)
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

bool EmulatedControlBoard::getRefPositions(double *refs)
{
    bool ok = true;

    for (int j = 0; j < m_axes; j++)
    {
        ok &= getRefPosition(j, &refs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool EmulatedControlBoard::getRefPositions(const int n_joint, const int *joints, double *refs)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getRefPosition(joints[i], &refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
