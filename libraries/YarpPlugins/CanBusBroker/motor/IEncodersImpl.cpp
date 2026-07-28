// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::getAxes(int *axes)
{
    *axes = deviceMapper.getControlledAxes();
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::resetEncoder(int j)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::resetEncoderRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::resetEncoders()
{
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::resetEncodersRaw);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setEncoder(int j, double val)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::setEncoderRaw, j, val);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setEncoders(const double * vals)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::setEncodersRaw, vals);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getEncoder(int j, double * v)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::getEncoderRaw, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getEncoders(double * encs)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::getEncodersRaw, encs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getEncoderSpeed(int j, double * sp)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::getEncoderSpeedRaw, j, sp);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getEncoderSpeeds(double * spds)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::getEncoderSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getEncoderAcceleration(int j, double * spds)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::getEncoderAccelerationRaw, j, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getEncoderAccelerations(double * accs)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::getEncoderAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getEncoderTimed(int j, double * enc, double * stamp)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersTimedRaw::getEncoderTimedRaw, j, enc, stamp);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getEncodersTimed(double * encs, double * stamps)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersTimedRaw::getEncodersTimedRaw, encs, stamps);
}

// -----------------------------------------------------------------------------
