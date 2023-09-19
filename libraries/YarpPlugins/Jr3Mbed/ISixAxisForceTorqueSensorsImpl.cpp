// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <yarp/os/SystemClock.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t Jr3Mbed::getNrOfSixAxisForceTorqueSensors() const
{
    return 1;
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status Jr3Mbed::getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const
{
    return yarp::dev::MAS_OK;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
{
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
    rxMutex.lock();
    out = {fx, fy, fz, mx, my, mz};
    rxMutex.unlock();
    timestamp = yarp::os::SystemClock::nowSystem();
    return true;
}

// -----------------------------------------------------------------------------
