// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;
using sensor_t = yarp::dev::ITemperatureSensors;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfTemperatureSensors() const
{
    yCTrace(CBB, "");
    return deviceMapper.getConnectedSensors<yarp::dev::ITemperatureSensors>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getTemperatureSensorStatus(std::size_t sens_index) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorStatus(&yarp::dev::ITemperatureSensors::getTemperatureSensorStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTemperatureSensorName(std::size_t sens_index, std::string & name) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::ITemperatureSensors::getTemperatureSensorName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTemperatureSensorFrameName(std::size_t sens_index, std::string & frameName) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::ITemperatureSensors::getTemperatureSensorFrameName, sens_index, frameName);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTemperatureSensorMeasure(std::size_t sens_index, double & out, double & timestamp) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput<sensor_t>(&yarp::dev::ITemperatureSensors::getTemperatureSensorMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTemperatureSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput<sensor_t>(&yarp::dev::ITemperatureSensors::getTemperatureSensorMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------