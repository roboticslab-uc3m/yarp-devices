// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getNumberOfMotors(int * ax)
{
    CD_DEBUG("\n");
    return getAxes(ax);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTemperature(int m, double * val)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getTemperatureRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTemperatures(double * vals)
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorRaw::getTemperaturesRaw, vals);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTemperatureLimit(int m, double * temp)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getTemperatureLimitRaw, m, temp);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setTemperatureLimit(int m, double temp)
{
    CD_DEBUG("(%d, %f)\n", m, temp);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::setTemperatureLimitRaw, m, temp);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getGearboxRatio(int m, double * val)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getGearboxRatioRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setGearboxRatio(int m, double val)
{
    CD_DEBUG("(%d, %f)\n", m, val);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::setGearboxRatioRaw, m, val);
}

// -----------------------------------------------------------------------------
