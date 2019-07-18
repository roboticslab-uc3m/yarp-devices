// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraSerialControlboard.hpp"

#include <ColorDebug.h>

// ------------------- IControlLimits Related ------------------------------------

bool roboticslab::DextraSerialControlboard::setLimits(int axis, double min, double max)
{
    CD_INFO("(%d, %f, %f)\n", axis, min, max);
    CHECK_JOINT(axis);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getLimits(int axis, double *min, double *max)
{
    CD_INFO("(%d)\n", axis);
    CHECK_JOINT(axis);

    std::pair<Synapse::setpoint_t, Synapse::setpoint_t> limits = Synapse::LIMITS[axis];
    *min = limits.first;
    *max = limits.second;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::setVelLimits(int axis, double min, double max)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getVelLimits(int axis, double *min, double *max)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------
