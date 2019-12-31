// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::setLimits(int axis, double min, double max)
{
    CD_DEBUG("(%d, %f, %f)\n", axis, min, max);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::setLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getLimits(int axis, double * min, double * max)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::getLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setVelLimits(int axis, double min, double max)
{
    CD_DEBUG("(%d, %f, %f)\n", axis, min, max);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::setVelLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getVelLimits(int axis, double * min, double * max)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::getVelLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------
