// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#define CHECK_SENSOR(idx, ret) do { int n = getNrOfPositionSensors(); if ((idx) < 0 || (idx) > n - 1) return ret; } while (0)

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfPositionSensors() const
{
    return deviceMapper.getConnectedSensors<yarp::dev::IPositionSensors>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getPositionSensorStatus(std::size_t sens_index) const
{
    CHECK_SENSOR(sens_index, yarp::dev::MAS_ERROR);
    return deviceMapper.getSensorStatus(&yarp::dev::IPositionSensors::getPositionSensorStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPositionSensorName(std::size_t sens_index, std::string & name) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IPositionSensors::getPositionSensorName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPositionSensorFrameName(std::size_t sens_index, std::string & frameName) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IPositionSensors::getPositionSensorFrameName, sens_index, frameName);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getPositionSensorMeasure(std::size_t sens_index, yarp::sig::Vector & xyz, double & timestamp) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IPositionSensors::getPositionSensorMeasure, sens_index, xyz, timestamp);
}

// -----------------------------------------------------------------------------
