// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefTorque(int j, double * t)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::getRefTorqueRaw, j, t);
}

bool CanBusControlboard::getRefTorques(double * t)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::ITorqueControlRaw::getRefTorquesRaw, t);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefTorque(int j, double t)
{
    yCTrace(CBCB, "%d %f", j, t);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::setRefTorqueRaw, j, t);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefTorques(const double * t)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::ITorqueControlRaw::setRefTorquesRaw, t);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefTorques(int n_joint, const int * joints, const double * t)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapJointGroup(&yarp::dev::ITorqueControlRaw::setRefTorquesRaw, n_joint, joints, t);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters * params)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::getMotorTorqueParamsRaw, j, params);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::setMotorTorqueParamsRaw, j, params);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTorque(int j, double * t)
{
    yCTrace(CBCB, "%d",j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::getTorqueRaw, j, t);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTorques(double * t)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::ITorqueControlRaw::getTorquesRaw, t);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTorqueRange(int j, double * min, double * max)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::getTorqueRangeRaw, j, min, max);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTorqueRanges(double * mins, double * maxs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::ITorqueControlRaw::getTorqueRangesRaw, mins, maxs);
}

// -----------------------------------------------------------------------------
