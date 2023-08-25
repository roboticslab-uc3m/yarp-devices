// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getAxes(int *axes)
{
    yCTrace(CBCB, "");
    *axes = deviceMapper.getControlledAxes();
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::resetEncoder(int j)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::resetEncoderRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::resetEncoders()
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::resetEncodersRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setEncoder(int j, double val)
{
    yCTrace(CBCB, "%d %f", j, val);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::setEncoderRaw, j, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setEncoders(const double * vals)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::setEncodersRaw, vals);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getEncoder(int j, double * v)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::getEncoderRaw, j, v);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getEncoders(double * encs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::getEncodersRaw, encs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getEncoderSpeed(int j, double * sp)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::getEncoderSpeedRaw, j, sp);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getEncoderSpeeds(double * spds)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::getEncoderSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getEncoderAcceleration(int j, double * spds)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersRaw::getEncoderAccelerationRaw, j, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getEncoderAccelerations(double * accs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersRaw::getEncoderAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getEncoderTimed(int j, double * enc, double * stamp)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IEncodersTimedRaw::getEncoderTimedRaw, j, enc, stamp);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getEncodersTimed(double * encs, double * stamps)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IEncodersTimedRaw::getEncodersTimedRaw, encs, stamps);
}

// -----------------------------------------------------------------------------
