// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getNumberOfMotorEncoders(int * num)
{
    yCTrace(CBCB, "");
    return getAxes(num);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::resetMotorEncoder(int m)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::resetMotorEncoderRaw, m);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::resetMotorEncoders()
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::resetMotorEncodersRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setMotorEncoderCountsPerRevolution(int m, double cpr)
{
    yCTrace(CBCB, "%d %f", m, cpr);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::setMotorEncoderCountsPerRevolutionRaw, m, cpr);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getMotorEncoderCountsPerRevolution(int m, double * cpr)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderCountsPerRevolutionRaw, m, cpr);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setMotorEncoder(int m, double val)
{
    yCTrace(CBCB, "%d %f", m, val);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::setMotorEncoderRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setMotorEncoders(const double * vals)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::setMotorEncodersRaw, vals);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getMotorEncoder(int m, double * v)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderRaw, m, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getMotorEncoders(double * encs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncodersRaw, encs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getMotorEncoderTimed(int m, double * enc, double * stamp)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderTimedRaw, m, enc, stamp);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getMotorEncodersTimed(double * encs, double * stamps)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncodersTimedRaw, encs, stamps);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getMotorEncoderSpeed(int m, double * sp)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedRaw, m, sp);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getMotorEncoderSpeeds(double *spds)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getMotorEncoderAcceleration(int m, double * acc)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationRaw, m, acc);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getMotorEncoderAccelerations(double * accs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------
