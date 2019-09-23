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
    return deviceMapper.singleJointMapping(m, val, &yarp::dev::IMotorRaw::getTemperatureRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTemperatures(double * vals)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(vals, &yarp::dev::IMotorRaw::getTemperaturesRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTemperatureLimit(int m, double * temp)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, temp, &yarp::dev::IMotorRaw::getTemperatureLimitRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setTemperatureLimit(int m, double temp)
{
    CD_DEBUG("(%d, %f)\n", m, temp);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, temp, &yarp::dev::IMotorRaw::setTemperatureLimitRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getGearboxRatio(int m, double * val)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, val, &yarp::dev::IMotorRaw::getGearboxRatioRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setGearboxRatio(int m, double val)
{
    CD_DEBUG("(%d, %f)\n", m, val);
    CHECK_JOINT(m);
    return deviceMapper.singleJointMapping(m, val, &yarp::dev::IMotorRaw::setGearboxRatioRaw);
}

// -----------------------------------------------------------------------------
