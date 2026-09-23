// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getRefTorque(int j, double * t)
#else
bool CanBusBroker::getRefTorque(int j, double * t)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::getRefTorqueRaw, j, t);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getRefTorques(double * t)
#else
bool CanBusBroker::getRefTorques(double * t)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::ITorqueControlRaw::getRefTorquesRaw, t);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setRefTorque(int j, double t)
#else
bool CanBusBroker::setRefTorque(int j, double t)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::setRefTorqueRaw, j, t);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setRefTorques(const double * t)
#else
bool CanBusBroker::setRefTorques(const double * t)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::ITorqueControlRaw::setRefTorquesRaw, t);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setRefTorques(int n_joint, const int * joints, const double * t)
#else
bool CanBusBroker::setRefTorques(int n_joint, const int * joints, const double * t)
#endif
{
    return deviceMapper.mapJointGroup(&yarp::dev::ITorqueControlRaw::setRefTorquesRaw, n_joint, joints, t);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters * params)
#else
bool CanBusBroker::getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters * params)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::getMotorTorqueParamsRaw, j, params);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
#else
bool CanBusBroker::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::setMotorTorqueParamsRaw, j, params);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTorque(int j, double * t)
#else
bool CanBusBroker::getTorque(int j, double * t)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::getTorqueRaw, j, t);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTorques(double * t)
#else
bool CanBusBroker::getTorques(double * t)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::ITorqueControlRaw::getTorquesRaw, t);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTorqueRange(int j, double * min, double * max)
#else
bool CanBusBroker::getTorqueRange(int j, double * min, double * max)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::ITorqueControlRaw::getTorqueRangeRaw, j, min, max);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTorqueRanges(double * mins, double * maxs)
#else
bool CanBusBroker::getTorqueRanges(double * mins, double * maxs)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::ITorqueControlRaw::getTorqueRangesRaw, mins, maxs);
}

// -----------------------------------------------------------------------------
