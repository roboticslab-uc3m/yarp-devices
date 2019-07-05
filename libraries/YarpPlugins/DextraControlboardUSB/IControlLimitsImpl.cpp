// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <ColorDebug.h>

// ------------------- IControlLimits Related ------------------------------------

bool roboticslab::DextraControlboardUSB::setLimits(int axis, double min, double max)
{
    CD_INFO("(%d, %f, %f)\n", axis, min, max);
    CHECK_JOINT(axis);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getLimits(int axis, double *min, double *max)
{
    CD_INFO("(%d)\n", axis);
    CHECK_JOINT(axis);

    std::pair<Synapse::setpoint_t, Synapse::setpoint_t> limits = Synapse::LIMITS[axis];
    *min = limits.first;
    *max = limits.second;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setVelLimits(int axis, double min, double max)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getVelLimits(int axis, double *min, double *max)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------
