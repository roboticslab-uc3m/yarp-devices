// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

#define CHECK_SENSOR(idx, ret) do { int n = getNrOfThreeAxisMagnetometers(); if ((idx) < 0 || (idx) > n - 1) return ret; } while (0)

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfThreeAxisMagnetometers() const
{
    yCTrace(CBB, "");
    return deviceMapper.getConnectedSensors<yarp::dev::IThreeAxisMagnetometers>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getThreeAxisMagnetometerStatus(std::size_t sens_index) const
{
    yCTrace(CBB, "%zu", sens_index);
    CHECK_SENSOR(sens_index, yarp::dev::MAS_ERROR);
    return deviceMapper.getSensorStatus(&yarp::dev::IThreeAxisMagnetometers::getThreeAxisMagnetometerStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisMagnetometerName(std::size_t sens_index, std::string & name) const
{
    yCTrace(CBB, "%zu", sens_index);
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisMagnetometers::getThreeAxisMagnetometerName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisMagnetometerFrameName(std::size_t sens_index, std::string & frameName) const
{
    yCTrace(CBB, "%zu", sens_index);
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisMagnetometers::getThreeAxisMagnetometerFrameName, sens_index, frameName);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisMagnetometerMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    yCTrace(CBB, "%zu", sens_index);
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisMagnetometers::getThreeAxisMagnetometerMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------