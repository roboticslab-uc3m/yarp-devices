// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::getCurrent(int m, double * curr)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::getCurrentRaw, m, curr);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getCurrents(double * currs)
{
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::getCurrentsRaw, currs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getCurrentRange(int m, double * min, double * max)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::getCurrentRangeRaw, m, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getCurrentRanges(double * mins, double * maxs)
{
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::getCurrentRangesRaw, mins, maxs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefCurrent(int m, double curr)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::setRefCurrentRaw, m, curr);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefCurrents(const double * currs)
{
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::setRefCurrentsRaw, currs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefCurrents(int n_motor, const int * motors, const double * currs)
{
    return deviceMapper.mapJointGroup(&yarp::dev::ICurrentControlRaw::setRefCurrentsRaw, n_motor, motors, currs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefCurrent(int m, double * curr)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::getRefCurrentRaw, m, curr);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefCurrents(double * currs)
{
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::getRefCurrentsRaw, currs);
}

// -----------------------------------------------------------------------------
