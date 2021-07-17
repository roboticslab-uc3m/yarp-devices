// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getNumberOfMotors(int * ax)
{
    yCTrace(CBCB, "");
    return getAxes(ax);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTemperature(int m, double * val)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getTemperatureRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTemperatures(double * vals)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorRaw::getTemperaturesRaw, vals);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTemperatureLimit(int m, double * temp)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getTemperatureLimitRaw, m, temp);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setTemperatureLimit(int m, double temp)
{
    yCTrace(CBCB, "%d %f", m, temp);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::setTemperatureLimitRaw, m, temp);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getGearboxRatio(int m, double * val)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getGearboxRatioRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setGearboxRatio(int m, double val)
{
    yCTrace(CBCB, "%d %f", m, val);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::setGearboxRatioRaw, m, val);
}

// -----------------------------------------------------------------------------
