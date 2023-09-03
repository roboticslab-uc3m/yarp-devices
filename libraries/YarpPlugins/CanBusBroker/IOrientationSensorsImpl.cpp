// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfOrientationSensors() const
{
    yCTrace(CBB, "");
    return deviceMapper.getConnectedSensors<yarp::dev::IOrientationSensors>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getOrientationSensorStatus(std::size_t sens_index) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorStatus(&yarp::dev::IOrientationSensors::getOrientationSensorStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getOrientationSensorName(std::size_t sens_index, std::string & name) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IOrientationSensors::getOrientationSensorName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getOrientationSensorFrameName(std::size_t sens_index, std::string & frameName) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IOrientationSensors::getOrientationSensorFrameName, sens_index, frameName);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getOrientationSensorMeasureAsRollPitchYaw(std::size_t sens_index, yarp::sig::Vector & rpy, double & timestamp) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IOrientationSensors::getOrientationSensorMeasureAsRollPitchYaw, sens_index, rpy, timestamp);
}

// -----------------------------------------------------------------------------