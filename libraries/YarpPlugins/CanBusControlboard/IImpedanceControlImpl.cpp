// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Log.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getImpedance(int j, double * stiffness, double * damping)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::getImpedanceRaw, j, stiffness, damping);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setImpedance(int j, double stiffness, double damping)
{
    yTrace("%d %f %f", j, stiffness, damping);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::setImpedanceRaw, j, stiffness, damping);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setImpedanceOffset(int j, double offset)
{
    yTrace("%d %f", j, offset);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::setImpedanceOffsetRaw, j, offset);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getImpedanceOffset(int j, double * offset)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::getImpedanceOffsetRaw, j, offset);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getCurrentImpedanceLimit(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    auto fn = &yarp::dev::IImpedanceControlRaw::getCurrentImpedanceLimitRaw;
    return deviceMapper.mapSingleJoint(fn, j, min_stiff, max_stiff, min_damp, max_damp);
}

// -----------------------------------------------------------------------------
