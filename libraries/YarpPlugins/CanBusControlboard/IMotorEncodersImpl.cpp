// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getNumberOfMotorEncoders(int * num)
{
    yCTrace(CBCB, "");
    return getAxes(num);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetMotorEncoder(int m)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::resetMotorEncoderRaw, m);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetMotorEncoders()
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::resetMotorEncodersRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setMotorEncoderCountsPerRevolution(int m, double cpr)
{
    yCTrace(CBCB, "%d %f", m, cpr);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::setMotorEncoderCountsPerRevolutionRaw, m, cpr);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderCountsPerRevolution(int m, double * cpr)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderCountsPerRevolutionRaw, m, cpr);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setMotorEncoder(int m, double val)
{
    yCTrace(CBCB, "%d %f", m, val);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::setMotorEncoderRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setMotorEncoders(const double * vals)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::setMotorEncodersRaw, vals);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoder(int m, double * v)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderRaw, m, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoders(double * encs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncodersRaw, encs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderTimed(int m, double * enc, double * stamp)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderTimedRaw, m, enc, stamp);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncodersTimed(double * encs, double * stamps)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncodersTimedRaw, encs, stamps);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderSpeed(int m, double * sp)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedRaw, m, sp);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderSpeeds(double *spds)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderAcceleration(int m, double * acc)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationRaw, m, acc);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderAccelerations(double * accs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------
