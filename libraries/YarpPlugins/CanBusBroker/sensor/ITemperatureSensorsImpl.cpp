// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#define CHECK_SENSOR(idx, ret) do { int n = getNrOfTemperatureSensors(); if ((idx) < 0 || (idx) > n - 1) return ret; } while (0)

using namespace roboticslab;
using sensor_t = yarp::dev::ITemperatureSensors;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfTemperatureSensors() const
{
    return deviceMapper.getConnectedSensors<yarp::dev::ITemperatureSensors>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getTemperatureSensorStatus(std::size_t sens_index) const
{
    CHECK_SENSOR(sens_index, yarp::dev::MAS_ERROR);
    return deviceMapper.getSensorStatus(&yarp::dev::ITemperatureSensors::getTemperatureSensorStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTemperatureSensorName(std::size_t sens_index, std::string & name) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::ITemperatureSensors::getTemperatureSensorName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTemperatureSensorFrameName(std::size_t sens_index, std::string & frameName) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::ITemperatureSensors::getTemperatureSensorFrameName, sens_index, frameName);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTemperatureSensorMeasure(std::size_t sens_index, double & out, double & timestamp) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput<sensor_t>(&yarp::dev::ITemperatureSensors::getTemperatureSensorMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTemperatureSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput<sensor_t>(&yarp::dev::ITemperatureSensors::getTemperatureSensorMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------
