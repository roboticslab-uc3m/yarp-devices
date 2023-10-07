// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <algorithm> // std::transform
#include <functional> // std::multiplies

#define CHECK_SENSOR(idx, ret) do { int n = getNrOfSixAxisForceTorqueSensors(); if ((idx) < 0 || (idx) > n - 1) return ret; } while (0)

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t Jr3Mbed::getNrOfSixAxisForceTorqueSensors() const
{
    return 1;
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status Jr3Mbed::getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const
{
    CHECK_SENSOR(sens_index, yarp::dev::MAS_status::MAS_ERROR);
    return status;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
{
    CHECK_SENSOR(sens_index, false);
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
    CHECK_SENSOR(sens_index, false);
    out.resize(raw.size());
    std::lock_guard lock(mtx);
    std::transform(raw.cbegin(), raw.cend(), scales.cbegin(), out.begin(), std::multiplies<>{});
    timestamp = this->timestamp;
    return true;
}

// -----------------------------------------------------------------------------
