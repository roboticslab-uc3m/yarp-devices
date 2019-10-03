// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getNumberOfMotorEncoders(int * num)
{
    CD_DEBUG("\n");
    return getAxes(num);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetMotorEncoder(int m)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::resetMotorEncoderRaw, m);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetMotorEncoders()
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::resetMotorEncodersRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setMotorEncoderCountsPerRevolution(int m, double cpr)
{
    CD_DEBUG("(%d, %f)\n", m, cpr);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::setMotorEncoderCountsPerRevolutionRaw, m, cpr);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderCountsPerRevolution(int m, double * cpr)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderCountsPerRevolutionRaw, m, cpr);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setMotorEncoder(int m, double val)
{
    CD_DEBUG("(%d, %f)\n", m, val);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::setMotorEncoderRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setMotorEncoders(const double * vals)
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::setMotorEncodersRaw, vals);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoder(int m, double * v)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderRaw, m, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoders(double * encs)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncodersRaw, encs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderTimed(int m, double * enc, double * stamp)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderTimedRaw, m, enc, stamp);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncodersTimed(double * encs, double * stamps)
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncodersTimedRaw, encs, stamps);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderSpeed(int m, double * sp)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedRaw, m, sp);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderSpeeds(double *spds)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderAcceleration(int m, double * acc)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationRaw, m, acc);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getMotorEncoderAccelerations(double * accs)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------
