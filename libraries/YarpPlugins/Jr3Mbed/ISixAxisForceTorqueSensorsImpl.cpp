// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <algorithm> // std::transform
#include <functional> // std::multiplies

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t Jr3Mbed::getNrOfSixAxisForceTorqueSensors() const
{
    return 1;
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status Jr3Mbed::getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const
{
    std::lock_guard lock(rxMutex);
    return status;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
{
    name = this->name;
    return true;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const
{
    return getSixAxisForceTorqueSensorName(sens_index, name);
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    std::lock_guard lock(rxMutex);
    std::transform(raw.cbegin(), raw.cend(), scales.cbegin(), out.begin(), std::multiplies<>{});
    timestamp = this->timestamp;
    return true;
}

// -----------------------------------------------------------------------------
