// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::getNumberOfMotorEncoders(int * num)
{
    yCTrace(CBB, "");
    return getAxes(num);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::resetMotorEncoder(int m)
{
    yCTrace(CBB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::resetMotorEncoderRaw, m);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::resetMotorEncoders()
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::resetMotorEncodersRaw);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setMotorEncoderCountsPerRevolution(int m, double cpr)
{
    yCTrace(CBB, "%d %f", m, cpr);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::setMotorEncoderCountsPerRevolutionRaw, m, cpr);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoderCountsPerRevolution(int m, double * cpr)
{
    yCTrace(CBB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderCountsPerRevolutionRaw, m, cpr);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setMotorEncoder(int m, double val)
{
    yCTrace(CBB, "%d %f", m, val);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::setMotorEncoderRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setMotorEncoders(const double * vals)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::setMotorEncodersRaw, vals);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoder(int m, double * v)
{
    yCTrace(CBB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderRaw, m, v);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoders(double * encs)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncodersRaw, encs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoderTimed(int m, double * enc, double * stamp)
{
    yCTrace(CBB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderTimedRaw, m, enc, stamp);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncodersTimed(double * encs, double * stamps)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncodersTimedRaw, encs, stamps);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoderSpeed(int m, double * sp)
{
    yCTrace(CBB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedRaw, m, sp);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoderSpeeds(double *spds)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoderAcceleration(int m, double * acc)
{
    yCTrace(CBB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationRaw, m, acc);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getMotorEncoderAccelerations(double * accs)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------
