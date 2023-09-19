// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfPositionSensors() const
{
    yCTrace(CBB, "");
    return deviceMapper.getConnectedSensors<yarp::dev::IPositionSensors>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getPositionSensorStatus(std::size_t sens_index) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorStatus(&yarp::dev::IPositionSensors::getPositionSensorStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPositionSensorName(std::size_t sens_index, std::string & name) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IPositionSensors::getPositionSensorName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPositionSensorFrameName(std::size_t sens_index, std::string & frameName) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IPositionSensors::getPositionSensorFrameName, sens_index, frameName);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPositionSensorMeasure(std::size_t sens_index, yarp::sig::Vector & xyz, double & timestamp) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IPositionSensors::getPositionSensorMeasure, sens_index, xyz, timestamp);
}

// -----------------------------------------------------------------------------
