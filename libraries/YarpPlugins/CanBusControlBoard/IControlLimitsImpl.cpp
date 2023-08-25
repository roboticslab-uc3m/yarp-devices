// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setLimits(int axis, double min, double max)
{
    yCTrace(CBCB, "%d %f %f", axis, min, max);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::setLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getLimits(int axis, double * min, double * max)
{
    yCTrace(CBCB, "%d", axis);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::getLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setVelLimits(int axis, double min, double max)
{
    yCTrace(CBCB, "%d %f %f", axis, min, max);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::setVelLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getVelLimits(int axis, double * min, double * max)
{
    yCTrace(CBCB, "%d", axis);
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::getVelLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------
