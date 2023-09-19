// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::getImpedance(int j, double * stiffness, double * damping)
{
    yCTrace(CBB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::getImpedanceRaw, j, stiffness, damping);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setImpedance(int j, double stiffness, double damping)
{
    yCTrace(CBB, "%d %f %f", j, stiffness, damping);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::setImpedanceRaw, j, stiffness, damping);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setImpedanceOffset(int j, double offset)
{
    yCTrace(CBB, "%d %f", j, offset);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::setImpedanceOffsetRaw, j, offset);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getImpedanceOffset(int j, double * offset)
{
    yCTrace(CBB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::getImpedanceOffsetRaw, j, offset);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getCurrentImpedanceLimit(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp)
{
    yCTrace(CBB, "%d", j);
    CHECK_JOINT(j);
    auto fn = &yarp::dev::IImpedanceControlRaw::getCurrentImpedanceLimitRaw;
    return deviceMapper.mapSingleJoint(fn, j, min_stiff, max_stiff, min_damp, max_damp);
}

// -----------------------------------------------------------------------------
