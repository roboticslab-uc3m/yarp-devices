// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getCurrent(int m, double * curr)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::getCurrentRaw, m, curr);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getCurrents(double * currs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::getCurrentsRaw, currs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getCurrentRange(int m, double * min, double * max)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::getCurrentRangeRaw, m, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getCurrentRanges(double * mins, double * maxs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::getCurrentRangesRaw, mins, maxs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setRefCurrent(int m, double curr)
{
    yCTrace(CBCB, "%d %f", m, curr);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::setRefCurrentRaw, m, curr);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setRefCurrents(const double * currs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::setRefCurrentsRaw, currs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setRefCurrents(int n_motor, const int * motors, const double * currs)
{
    yCTrace(CBCB, "%d", n_motor);
    return deviceMapper.mapJointGroup(&yarp::dev::ICurrentControlRaw::setRefCurrentsRaw, n_motor, motors, currs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefCurrent(int m, double * curr)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::ICurrentControlRaw::getRefCurrentRaw, m, curr);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefCurrents(double * currs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::ICurrentControlRaw::getRefCurrentsRaw, currs);
}

// -----------------------------------------------------------------------------
