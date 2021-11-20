// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <yarp/conf/version.h>
#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool DextraRawControlboard::setLimitsRaw(int axis, double min, double max)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(DEXTRA, id(), "%d %f %f", axis, min, max);
#else
    yCTrace(DEXTRA, "%d %f %f", axis, min, max);
#endif
    CHECK_JOINT(axis);
    return false;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getLimitsRaw(int axis, double * min, double * max)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(DEXTRA, id(), "%d", axis);
#else
    yCTrace(DEXTRA, "%d", axis);
#endif
    CHECK_JOINT(axis);

    auto limits = Synapse::LIMITS[axis];
    *min = limits.first;
    *max = limits.second;

    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::setVelLimitsRaw(int axis, double min, double max)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCIWarning(DEXTRA, id(), "setVelLimitsRaw() not supported");
#else
    yCWarning(DEXTRA, "setVelLimitsRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getVelLimitsRaw(int axis, double * min, double * max)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCIWarning(DEXTRA, id(), "getVelLimitsRaw() not supported");
#else
    yCWarning(DEXTRA, "getVelLimitsRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------
