// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getCurrent(int m, double * curr)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::getCurrentRaw, m, curr);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getCurrents(double * currs)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::getCurrentsRaw, currs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getCurrentRange(int m, double * min, double * max)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::getCurrentRangeRaw, m, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getCurrentRanges(double * mins, double * maxs)
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::getCurrentRangesRaw, mins, maxs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefCurrent(int m, double curr)
{
    CD_DEBUG("(%d, %f)\n", m, curr);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::setRefCurrentRaw, m, curr);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefCurrents(const double * currs)
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::setRefCurrentsRaw, currs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefCurrents(int n_motor, const int * motors, const double * currs)
{
    CD_DEBUG("(%d)\n", n_motor);
    return deviceMapper.mapJointGroup(&yarp::dev::ICurrentControlRaw::setRefCurrentsRaw, n_motor, motors, currs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefCurrent(int m, double * curr)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::getRefCurrentRaw, m, curr);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefCurrents(double * currs)
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::getRefCurrentsRaw, currs);
}

// -----------------------------------------------------------------------------
