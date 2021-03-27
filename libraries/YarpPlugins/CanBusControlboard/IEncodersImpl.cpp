// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Log.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getAxes(int *axes)
{
    yTrace("");
    *axes = deviceMapper.getControlledAxes();
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetEncoder(int j)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::resetEncoderRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetEncoders()
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::resetEncodersRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setEncoder(int j, double val)
{
    yTrace("%d %f", j, val);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::setEncoderRaw, j, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setEncoders(const double * vals)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::setEncodersRaw, vals);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoder(int j, double * v)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::getEncoderRaw, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoders(double * encs)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::getEncodersRaw, encs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderSpeed(int j, double * sp)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::getEncoderSpeedRaw, j, sp);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderSpeeds(double * spds)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::getEncoderSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderAcceleration(int j, double * spds)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::getEncoderAccelerationRaw, j, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderAccelerations(double * accs)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::getEncoderAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderTimed(int j, double * enc, double * stamp)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersTimedRaw::getEncoderTimedRaw, j, enc, stamp);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncodersTimed(double * encs, double * stamps)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersTimedRaw::getEncodersTimedRaw, encs, stamps);
}

// -----------------------------------------------------------------------------
