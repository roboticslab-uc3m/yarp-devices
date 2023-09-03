// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

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
    return deviceMapper.getSensorStatus(&yarp::dev::IThreeAxisMagnetometers::getThreeAxisMagnetometerStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisMagnetometerName(std::size_t sens_index, std::string & name) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisMagnetometers::getThreeAxisMagnetometerName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisMagnetometerFrameName(std::size_t sens_index, std::string & frameName) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisMagnetometers::getThreeAxisMagnetometerFrameName, sens_index, frameName);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisMagnetometerMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisMagnetometers::getThreeAxisMagnetometerMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------