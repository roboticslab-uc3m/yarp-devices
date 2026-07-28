// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::getNumberOfMotorEncoders(int * num)
{
    return getAxes(num);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::resetMotorEncoder(int m)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::resetMotorEncoderRaw, m);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::resetMotorEncoders()
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::resetMotorEncodersRaw);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setMotorEncoderCountsPerRevolution(int m, double cpr)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::setMotorEncoderCountsPerRevolutionRaw, m, cpr);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoderCountsPerRevolution(int m, double * cpr)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderCountsPerRevolutionRaw, m, cpr);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setMotorEncoder(int m, double val)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::setMotorEncoderRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setMotorEncoders(const double * vals)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::setMotorEncodersRaw, vals);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoder(int m, double * v)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderRaw, m, v);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoders(double * encs)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncodersRaw, encs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoderTimed(int m, double * enc, double * stamp)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderTimedRaw, m, enc, stamp);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncodersTimed(double * encs, double * stamps)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncodersTimedRaw, encs, stamps);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoderSpeed(int m, double * sp)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedRaw, m, sp);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoderSpeeds(double *spds)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoderAcceleration(int m, double * acc)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationRaw, m, acc);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoderAccelerations(double * accs)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------
