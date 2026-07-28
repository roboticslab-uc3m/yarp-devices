// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#define CHECK_SENSOR(idx, ret) do { int n = getNrOfThreeAxisLinearAccelerometers(); if ((idx) < 0 || (idx) > n - 1) return ret; } while (0)

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfThreeAxisLinearAccelerometers() const
{
    return deviceMapper.getConnectedSensors<yarp::dev::IThreeAxisLinearAccelerometers>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getThreeAxisLinearAccelerometerStatus(std::size_t sens_index) const
{
    CHECK_SENSOR(sens_index, yarp::dev::MAS_ERROR);
    return deviceMapper.getSensorStatus(&yarp::dev::IThreeAxisLinearAccelerometers::getThreeAxisLinearAccelerometerStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisLinearAccelerometerName(std::size_t sens_index, std::string & name) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisLinearAccelerometers::getThreeAxisLinearAccelerometerName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisLinearAccelerometerFrameName(std::size_t sens_index, std::string & frameName) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisLinearAccelerometers::getThreeAxisLinearAccelerometerFrameName, sens_index, frameName);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisLinearAccelerometerMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisLinearAccelerometers::getThreeAxisLinearAccelerometerMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------
