// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setRefDutyCycle(int m, double ref)
#else
bool CanBusBroker::setRefDutyCycle(int m, double ref)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPWMControlRaw::setRefDutyCycleRaw, m, ref);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setRefDutyCycles(const double * refs)
#else
bool CanBusBroker::setRefDutyCycles(const double * refs)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPWMControlRaw::setRefDutyCyclesRaw, refs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getRefDutyCycle(int m, double * ref)
#else
bool CanBusBroker::getRefDutyCycle(int m, double * ref)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPWMControlRaw::getRefDutyCycleRaw, m, ref);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getRefDutyCycles(double * refs)
#else
bool CanBusBroker::getRefDutyCycles(double * refs)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPWMControlRaw::getRefDutyCyclesRaw, refs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getDutyCycle(int m, double * val)
#else
bool CanBusBroker::getDutyCycle(int m, double * val)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPWMControlRaw::getDutyCycleRaw, m, val);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getDutyCycles(double * vals)
#else
bool CanBusBroker::getDutyCycles(double * vals)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPWMControlRaw::getDutyCyclesRaw, vals);
}

// -----------------------------------------------------------------------------
