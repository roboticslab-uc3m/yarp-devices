// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getNumberOfMotorEncoders(int * num)
#else
bool CanBusBroker::getNumberOfMotorEncoders(int * num)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return getAxes(*reinterpret_cast<std::size_t *>(num));
#else
    return getAxes(num);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::resetMotorEncoder(int m)
#else
bool CanBusBroker::resetMotorEncoder(int m)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::resetMotorEncoderRaw, m);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::resetMotorEncoders()
#else
bool CanBusBroker::resetMotorEncoders()
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::resetMotorEncodersRaw);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setMotorEncoderCountsPerRevolution(int m, double cpr)
#else
bool CanBusBroker::setMotorEncoderCountsPerRevolution(int m, double cpr)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::setMotorEncoderCountsPerRevolutionRaw, m, cpr);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getMotorEncoderCountsPerRevolution(int m, double * cpr)
#else
bool CanBusBroker::getMotorEncoderCountsPerRevolution(int m, double * cpr)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderCountsPerRevolutionRaw, m, cpr);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setMotorEncoder(int m, double val)
#else
bool CanBusBroker::setMotorEncoder(int m, double val)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::setMotorEncoderRaw, m, val);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setMotorEncoders(const double * vals)
#else
bool CanBusBroker::setMotorEncoders(const double * vals)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::setMotorEncodersRaw, vals);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getMotorEncoder(int m, double * v)
#else
bool CanBusBroker::getMotorEncoder(int m, double * v)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderRaw, m, v);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getMotorEncoders(double * encs)
#else
bool CanBusBroker::getMotorEncoders(double * encs)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncodersRaw, encs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getMotorEncoderTimed(int m, double * enc, double * stamp)
#else
bool CanBusBroker::getMotorEncoderTimed(int m, double * enc, double * stamp)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderTimedRaw, m, enc, stamp);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getMotorEncodersTimed(double * encs, double * stamps)
#else
bool CanBusBroker::getMotorEncodersTimed(double * encs, double * stamps)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncodersTimedRaw, encs, stamps);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getMotorEncoderSpeed(int m, double * sp)
#else
bool CanBusBroker::getMotorEncoderSpeed(int m, double * sp)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedRaw, m, sp);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getMotorEncoderSpeeds(double *spds)
#else
bool CanBusBroker::getMotorEncoderSpeeds(double *spds)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncoderSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getMotorEncoderAcceleration(int m, double * acc)
#else
bool CanBusBroker::getMotorEncoderAcceleration(int m, double * acc)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationRaw, m, acc);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getMotorEncoderAccelerations(double * accs)
#else
bool CanBusBroker::getMotorEncoderAccelerations(double * accs)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorEncodersRaw::getMotorEncoderAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------
