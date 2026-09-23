// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getCurrent(int m, double * curr)
#else
bool CanBusBroker::getCurrent(int m, double * curr)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::getCurrentRaw, m, curr);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getCurrents(double * currs)
#else
bool CanBusBroker::getCurrents(double * currs)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::getCurrentsRaw, currs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getCurrentRange(int m, double * min, double * max)
#else
bool CanBusBroker::getCurrentRange(int m, double * min, double * max)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::getCurrentRangeRaw, m, min, max);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getCurrentRanges(double * mins, double * maxs)
#else
bool CanBusBroker::getCurrentRanges(double * mins, double * maxs)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::getCurrentRangesRaw, mins, maxs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setRefCurrent(int m, double curr)
#else
bool CanBusBroker::setRefCurrent(int m, double curr)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::setRefCurrentRaw, m, curr);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setRefCurrents(const double * currs)
#else
bool CanBusBroker::setRefCurrents(const double * currs)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::setRefCurrentsRaw, currs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setRefCurrents(int n_motor, const int * motors, const double * currs)
#else
bool CanBusBroker::setRefCurrents(int n_motor, const int * motors, const double * currs)
#endif
{
    return deviceMapper.mapJointGroup(&yarp::dev::ICurrentControlRaw::setRefCurrentsRaw, n_motor, motors, currs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getRefCurrent(int m, double * curr)
#else
bool CanBusBroker::getRefCurrent(int m, double * curr)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::getRefCurrentRaw, m, curr);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getRefCurrents(double * currs)
#else
bool CanBusBroker::getRefCurrents(double * currs)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::getRefCurrentsRaw, currs);
}

// -----------------------------------------------------------------------------
