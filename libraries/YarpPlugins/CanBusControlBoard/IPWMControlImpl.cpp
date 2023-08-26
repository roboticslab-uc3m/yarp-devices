// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setRefDutyCycle(int m, double ref)
{
    yCTrace(CBCB, "%d %f", m, ref);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPWMControlRaw::setRefDutyCycleRaw, m, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setRefDutyCycles(const double * refs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPWMControlRaw::setRefDutyCyclesRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefDutyCycle(int m, double * ref)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPWMControlRaw::getRefDutyCycleRaw, m, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefDutyCycles(double * refs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPWMControlRaw::getRefDutyCyclesRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getDutyCycle(int m, double * val)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPWMControlRaw::getDutyCycleRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getDutyCycles(double * vals)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPWMControlRaw::getDutyCyclesRaw, vals);
}

// -----------------------------------------------------------------------------
