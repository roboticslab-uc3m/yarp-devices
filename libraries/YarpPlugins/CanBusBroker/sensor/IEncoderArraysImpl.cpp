// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfEncoderArrays() const
{
    yCTrace(CBB, "");
    return deviceMapper.getConnectedSensors<yarp::dev::IEncoderArrays>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getEncoderArrayStatus(std::size_t sens_index) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorStatus(&yarp::dev::IEncoderArrays::getEncoderArrayStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getEncoderArrayName(std::size_t sens_index, std::string & name) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IEncoderArrays::getEncoderArrayName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getEncoderArrayMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IEncoderArrays::getEncoderArrayMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getEncoderArraySize(std::size_t sens_index) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorArraySize(&yarp::dev::IEncoderArrays::getEncoderArraySize, sens_index);
}

// -----------------------------------------------------------------------------
