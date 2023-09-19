// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefTorque(int j, double * t)
{
    yCTrace(CBB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::getRefTorqueRaw, j, t);
}

bool CanBusBroker::getRefTorques(double * t)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::ITorqueControlRaw::getRefTorquesRaw, t);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefTorque(int j, double t)
{
    yCTrace(CBB, "%d %f", j, t);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::setRefTorqueRaw, j, t);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefTorques(const double * t)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::ITorqueControlRaw::setRefTorquesRaw, t);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefTorques(int n_joint, const int * joints, const double * t)
{
    yCTrace(CBB, "");
    return deviceMapper.mapJointGroup(&yarp::dev::ITorqueControlRaw::setRefTorquesRaw, n_joint, joints, t);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters * params)
{
    yCTrace(CBB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::getMotorTorqueParamsRaw, j, params);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
{
    yCTrace(CBB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::setMotorTorqueParamsRaw, j, params);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTorque(int j, double * t)
{
    yCTrace(CBB, "%d",j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::getTorqueRaw, j, t);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTorques(double * t)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::ITorqueControlRaw::getTorquesRaw, t);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTorqueRange(int j, double * min, double * max)
{
    yCTrace(CBB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::getTorqueRangeRaw, j, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTorqueRanges(double * mins, double * maxs)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::ITorqueControlRaw::getTorqueRangesRaw, mins, maxs);
}

// -----------------------------------------------------------------------------
