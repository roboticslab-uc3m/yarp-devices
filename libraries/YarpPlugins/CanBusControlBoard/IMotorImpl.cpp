// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getNumberOfMotors(int * ax)
{
    yCTrace(CBCB, "");
    return getAxes(ax);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getTemperature(int m, double * val)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getTemperatureRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getTemperatures(double * vals)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IMotorRaw::getTemperaturesRaw, vals);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getTemperatureLimit(int m, double * temp)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getTemperatureLimitRaw, m, temp);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setTemperatureLimit(int m, double temp)
{
    yCTrace(CBCB, "%d %f", m, temp);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::setTemperatureLimitRaw, m, temp);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getGearboxRatio(int m, double * val)
{
    yCTrace(CBCB, "%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::getGearboxRatioRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setGearboxRatio(int m, double val)
{
    yCTrace(CBCB, "%d %f", m, val);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IMotorRaw::setGearboxRatioRaw, m, val);
}

// -----------------------------------------------------------------------------
