// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::getNumberOfMotors(int * ax)
{
    return getAxes(ax);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTemperature(int m, double * val)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getTemperatureRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTemperatures(double * vals)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorRaw::getTemperaturesRaw, vals);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTemperatureLimit(int m, double * temp)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getTemperatureLimitRaw, m, temp);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setTemperatureLimit(int m, double temp)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::setTemperatureLimitRaw, m, temp);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getGearboxRatio(int m, double * val)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getGearboxRatioRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setGearboxRatio(int m, double val)
{
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::setGearboxRatioRaw, m, val);
}

// -----------------------------------------------------------------------------
