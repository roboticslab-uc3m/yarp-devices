// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::setLimits(int axis, double min, double max)
{
    yCTrace(CBB, "%d %f %f", axis, min, max);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::setLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getLimits(int axis, double * min, double * max)
{
    yCTrace(CBB, "%d", axis);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::getLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setVelLimits(int axis, double min, double max)
{
    yCTrace(CBB, "%d %f %f", axis, min, max);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::setVelLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getVelLimits(int axis, double * min, double * max)
{
    yCTrace(CBB, "%d", axis);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::getVelLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------
