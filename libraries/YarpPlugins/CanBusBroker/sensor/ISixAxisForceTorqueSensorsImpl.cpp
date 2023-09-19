// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfSixAxisForceTorqueSensors() const
{
    yCTrace(CBB, "");
    return deviceMapper.getConnectedSensors<yarp::dev::ISixAxisForceTorqueSensors>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorStatus(&yarp::dev::ISixAxisForceTorqueSensors::getSixAxisForceTorqueSensorStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::ISixAxisForceTorqueSensors::getSixAxisForceTorqueSensorName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & frameName) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::ISixAxisForceTorqueSensors::getSixAxisForceTorqueSensorFrameName, sens_index, frameName);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::ISixAxisForceTorqueSensors::getSixAxisForceTorqueSensorMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------