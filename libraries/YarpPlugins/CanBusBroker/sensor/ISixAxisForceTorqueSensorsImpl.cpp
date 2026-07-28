// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#define CHECK_SENSOR(idx, ret) do { int n = getNrOfSixAxisForceTorqueSensors(); if ((idx) < 0 || (idx) > n - 1) return ret; } while (0)

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfSixAxisForceTorqueSensors() const
{
    return deviceMapper.getConnectedSensors<yarp::dev::ISixAxisForceTorqueSensors>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const
{
    CHECK_SENSOR(sens_index, yarp::dev::MAS_ERROR);
    return deviceMapper.getSensorStatus(&yarp::dev::ISixAxisForceTorqueSensors::getSixAxisForceTorqueSensorStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::ISixAxisForceTorqueSensors::getSixAxisForceTorqueSensorName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & frameName) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::ISixAxisForceTorqueSensors::getSixAxisForceTorqueSensorFrameName, sens_index, frameName);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::ISixAxisForceTorqueSensors::getSixAxisForceTorqueSensorMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------
