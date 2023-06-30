// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ForceTorqueCan.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t ForceTorqueCan::getNrOfSixAxisForceTorqueSensors() const
{
    return 1;
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status ForceTorqueCan::getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const
{
    return yarp::dev::MAS_OK;
}

// -----------------------------------------------------------------------------

bool ForceTorqueCan::getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
{
    return true;
}

// -----------------------------------------------------------------------------

bool ForceTorqueCan::getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const
{
    return getSixAxisForceTorqueSensorName(sens_index, name);
}

// -----------------------------------------------------------------------------

bool ForceTorqueCan::getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    out = canReadThreads[sens_index]->getMeasurements();
    timestamp = yarp::os::SystemClock::nowSystem();
    return true;
}

// -----------------------------------------------------------------------------
