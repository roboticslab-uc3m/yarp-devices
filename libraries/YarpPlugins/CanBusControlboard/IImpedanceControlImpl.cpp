// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getImpedance(int j, double * stiffness, double * damping)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::getImpedanceRaw, j, stiffness, damping);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setImpedance(int j, double stiffness, double damping)
{
    yCTrace(CBCB, "%d %f %f", j, stiffness, damping);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::setImpedanceRaw, j, stiffness, damping);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setImpedanceOffset(int j, double offset)
{
    yCTrace(CBCB, "%d %f", j, offset);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::setImpedanceOffsetRaw, j, offset);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getImpedanceOffset(int j, double * offset)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::getImpedanceOffsetRaw, j, offset);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getCurrentImpedanceLimit(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    auto fn = &yarp::dev::IImpedanceControlRaw::getCurrentImpedanceLimitRaw;
    return deviceMapper.mapSingleJoint(fn, j, min_stiff, max_stiff, min_damp, max_damp);
}

// -----------------------------------------------------------------------------
