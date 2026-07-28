// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#define CHECK_SENSOR(idx, ret) do { int n = getNrOfThreeAxisGyroscopes(); if ((idx) < 0 || (idx) > n - 1) return ret; } while (0)

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfThreeAxisGyroscopes() const
{
    return deviceMapper.getConnectedSensors<yarp::dev::IThreeAxisGyroscopes>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getThreeAxisGyroscopeStatus(std::size_t sens_index) const
{
    CHECK_SENSOR(sens_index, yarp::dev::MAS_ERROR);
    return deviceMapper.getSensorStatus(&yarp::dev::IThreeAxisGyroscopes::getThreeAxisGyroscopeStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisGyroscopeName(std::size_t sens_index, std::string & name) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisGyroscopes::getThreeAxisGyroscopeName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisGyroscopeFrameName(std::size_t sens_index, std::string & frameName) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisGyroscopes::getThreeAxisGyroscopeFrameName, sens_index, frameName);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getThreeAxisGyroscopeMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisGyroscopes::getThreeAxisGyroscopeMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------
