// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <ColorDebug.h>

// ------------------- IControlLimits Related ------------------------------------

bool roboticslab::DextraRawControlboard::setLimitsRaw(int axis, double min, double max)
{
    CD_INFO("(%d, %f, %f)\n", axis, min, max);
    CHECK_JOINT(axis);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::getLimitsRaw(int axis, double *min, double *max)
{
    CD_INFO("(%d)\n", axis);
    CHECK_JOINT(axis);

    std::pair<Synapse::setpoint_t, Synapse::setpoint_t> limits = Synapse::LIMITS[axis];
    *min = limits.first;
    *max = limits.second;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::setVelLimitsRaw(int axis, double min, double max)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::getVelLimitsRaw(int axis, double *min, double *max)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------
