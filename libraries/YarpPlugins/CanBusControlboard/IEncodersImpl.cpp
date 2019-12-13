// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getAxes(int *axes)
{
    CD_DEBUG("\n");
    *axes = deviceMapper.getControlledAxes();
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetEncoder(int j)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::resetEncoderRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::resetEncoders()
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::resetEncodersRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setEncoder(int j, double val)
{
    CD_DEBUG("(%d, %f)\n", j, val);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::setEncoderRaw, j, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setEncoders(const double * vals)
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::setEncodersRaw, vals);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoder(int j, double * v)
{
    CD_DEBUG("%d\n", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::getEncoderRaw, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoders(double * encs)
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::getEncodersRaw, encs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderSpeed(int j, double * sp)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::getEncoderSpeedRaw, j, sp);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderSpeeds(double * spds)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::getEncoderSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderAcceleration(int j, double * spds)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::getEncoderAccelerationRaw, j, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderAccelerations(double * accs)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::getEncoderAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncoderTimed(int j, double * enc, double * stamp)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersTimedRaw::getEncoderTimedRaw, j, enc, stamp);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getEncodersTimed(double * encs, double * stamps)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersTimedRaw::getEncodersTimedRaw, encs, stamps);
}

// -----------------------------------------------------------------------------
