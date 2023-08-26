// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::setLimitsRaw(int axis, double min, double max)
{
    yCITrace(DEXTRA, id(), "%d %f %f", axis, min, max);
    CHECK_JOINT(axis);
    return false;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getLimitsRaw(int axis, double * min, double * max)
{
    yCITrace(DEXTRA, id(), "%d", axis);
    CHECK_JOINT(axis);

    const auto & [_min, _max] = Synapse::LIMITS[axis];
    *min = _min;
    *max = _max;

    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::setVelLimitsRaw(int axis, double min, double max)
{
    yCIWarning(DEXTRA, id(), "setVelLimitsRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getVelLimitsRaw(int axis, double * min, double * max)
{
    yCIWarning(DEXTRA, id(), "getVelLimitsRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------
