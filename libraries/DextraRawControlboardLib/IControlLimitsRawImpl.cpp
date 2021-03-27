// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <yarp/os/Log.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool DextraRawControlboard::setLimitsRaw(int axis, double min, double max)
{
    yTrace("%d %f %f", axis, min, max);
    CHECK_JOINT(axis);
    return false;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getLimitsRaw(int axis, double * min, double * max)
{
    yTrace("%d", axis);
    CHECK_JOINT(axis);

    auto limits = Synapse::LIMITS[axis];
    *min = limits.first;
    *max = limits.second;

    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::setVelLimitsRaw(int axis, double min, double max)
{
    yWarning("setVelLimitsRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getVelLimitsRaw(int axis, double * min, double * max)
{
    yWarning("getVelLimitsRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------
