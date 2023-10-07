// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t LacqueyFetch::getNrOfSixAxisForceTorqueSensors() const
{
    if (sensor)
    {
        return sensor->getNrOfSixAxisForceTorqueSensors();
    }

    return 0;
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status LacqueyFetch::getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const
{
    if (sensor)
    {
        return sensor->getSixAxisForceTorqueSensorStatus(sens_index);
    }

    return yarp::dev::MAS_status::MAS_ERROR;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const
{
    if (sensor)
    {
        return sensor->getSixAxisForceTorqueSensorName(sens_index, name);
    }

    return false;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const
{
    return getSixAxisForceTorqueSensorName(sens_index, name);
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    if (sensor)
    {
        return sensor->getSixAxisForceTorqueSensorMeasure(sens_index, out, timestamp);
    }

    return false;
}

// -----------------------------------------------------------------------------
