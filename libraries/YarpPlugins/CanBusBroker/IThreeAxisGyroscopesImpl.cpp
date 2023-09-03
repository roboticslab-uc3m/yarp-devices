// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfThreeAxisGyroscopes() const
{
    yCTrace(CBB, "");
    return deviceMapper.getConnectedSensors<yarp::dev::IThreeAxisGyroscopes>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getThreeAxisGyroscopeStatus(std::size_t sens_index) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorStatus(&yarp::dev::IThreeAxisGyroscopes::getThreeAxisGyroscopeStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisGyroscopeName(std::size_t sens_index, std::string & name) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisGyroscopes::getThreeAxisGyroscopeName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisGyroscopeFrameName(std::size_t sens_index, std::string & frameName) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisGyroscopes::getThreeAxisGyroscopeFrameName, sens_index, frameName);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisGyroscopeMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisGyroscopes::getThreeAxisGyroscopeMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------