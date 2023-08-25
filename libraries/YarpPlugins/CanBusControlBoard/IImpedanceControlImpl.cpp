// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getImpedance(int j, double * stiffness, double * damping)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::getImpedanceRaw, j, stiffness, damping);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setImpedance(int j, double stiffness, double damping)
{
    yCTrace(CBCB, "%d %f %f", j, stiffness, damping);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::setImpedanceRaw, j, stiffness, damping);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setImpedanceOffset(int j, double offset)
{
    yCTrace(CBCB, "%d %f", j, offset);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::setImpedanceOffsetRaw, j, offset);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getImpedanceOffset(int j, double * offset)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::getImpedanceOffsetRaw, j, offset);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getCurrentImpedanceLimit(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    auto fn = &yarp::dev::IImpedanceControlRaw::getCurrentImpedanceLimitRaw;
    return deviceMapper.mapSingleJoint(fn, j, min_stiff, max_stiff, min_damp, max_damp);
}

// -----------------------------------------------------------------------------
