// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

#define CHECK_SENSOR(idx, ret) do { int n = getNrOfEncoderArrays(); if ((idx) < 0 || (idx) > n - 1) return ret; } while (0)

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
    CHECK_SENSOR(sens_index, yarp::dev::MAS_ERROR);
    return deviceMapper.getSensorStatus(&yarp::dev::IEncoderArrays::getEncoderArrayStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getEncoderArrayName(std::size_t sens_index, std::string & name) const
{
    yCTrace(CBB, "%zu", sens_index);
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IEncoderArrays::getEncoderArrayName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getEncoderArrayMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    yCTrace(CBB, "%zu", sens_index);
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::IEncoderArrays::getEncoderArrayMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getEncoderArraySize(std::size_t sens_index) const
{
    yCTrace(CBB, "%zu", sens_index);
    CHECK_SENSOR(sens_index, 0);
    return deviceMapper.getSensorArraySize(&yarp::dev::IEncoderArrays::getEncoderArraySize, sens_index);
}

// -----------------------------------------------------------------------------
