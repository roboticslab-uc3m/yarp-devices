// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <ColorDebug.h>

// ------------------- IPositionDirect Related --------------------------------

bool roboticslab::FakeControlboard::setPosition(int j, double ref)
{
    if ((unsigned int)j > axes)
    {
        CD_ERROR("axis index more than axes.\n");
        return false;
    }

    if (controlMode != POSITION_DIRECT_MODE)
    {
        CD_ERROR("will not setPosition as not in positionDirectMode\n");
        return false;
    }

    targetExposed[j] = ref;
    encRaw[j] = ref * encRawExposed[j];

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setPositions(const int n_joint, const int *joints, const double *refs)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setPosition(joints[i], refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setPositions(const double *refs)
{
    bool ok = true;

    for (int j = 0; j < axes; j++)
    {
        ok &= setPosition(j, refs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getRefPosition(const int joint, double *ref)
{
    if ((unsigned int)joint > axes)
    {
        CD_ERROR("axis index more than axes.\n");
        return false;
    }

    if (controlMode != POSITION_DIRECT_MODE)
    {
        CD_ERROR("will not getRefPosition as not in positionDirectMode\n");
        return false;
    }

    *ref = targetExposed[joint];

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getRefPositions(double *refs)
{
    bool ok = true;

    for (int j = 0; j < axes; j++)
    {
        ok &= getRefPosition(j, &refs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getRefPositions(const int n_joint, const int *joints, double *refs)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getRefPosition(joints[i], &refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
