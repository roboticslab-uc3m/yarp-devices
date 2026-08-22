// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getNumberOfMotors(int * ax)
#else
bool CanBusBroker::getNumberOfMotors(int * ax)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return getAxes(*reinterpret_cast<std::size_t *>(ax));
#else
    return getAxes(ax);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTemperature(int m, double * val)
#else
bool CanBusBroker::getTemperature(int m, double * val)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getTemperatureRaw, m, val);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTemperatures(double * vals)
#else
bool CanBusBroker::getTemperatures(double * vals)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorRaw::getTemperaturesRaw, vals);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTemperatureLimit(int m, double * temp)
#else
bool CanBusBroker::getTemperatureLimit(int m, double * temp)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getTemperatureLimitRaw, m, temp);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setTemperatureLimit(int m, double temp)
#else
bool CanBusBroker::setTemperatureLimit(int m, double temp)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::setTemperatureLimitRaw, m, temp);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getGearboxRatio(int m, double * val)
#else
bool CanBusBroker::getGearboxRatio(int m, double * val)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getGearboxRatioRaw, m, val);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setGearboxRatio(int m, double val)
#else
bool CanBusBroker::setGearboxRatio(int m, double val)
#endif
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::setGearboxRatioRaw, m, val);
}

// -----------------------------------------------------------------------------
