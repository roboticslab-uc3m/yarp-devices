// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefDutyCycle(int m, double ref)
{
    yCTrace(CBB, "%d %f", m, ref);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPWMControlRaw::setRefDutyCycleRaw, m, ref);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefDutyCycles(const double * refs)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPWMControlRaw::setRefDutyCyclesRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefDutyCycle(int m, double * ref)
{
    yCTrace(CBB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPWMControlRaw::getRefDutyCycleRaw, m, ref);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefDutyCycles(double * refs)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPWMControlRaw::getRefDutyCyclesRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getDutyCycle(int m, double * val)
{
    yCTrace(CBB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPWMControlRaw::getDutyCycleRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getDutyCycles(double * vals)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPWMControlRaw::getDutyCyclesRaw, vals);
}

// -----------------------------------------------------------------------------
