// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

#define CHECK_SENSOR(idx, ret) do { int n = getNrOfSkinPatches(); if ((idx) < 0 || (idx) > n - 1) return ret; } while (0)

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfSkinPatches() const
{
    yCTrace(CBB, "");
    return deviceMapper.getConnectedSensors<yarp::dev::ISkinPatches>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getSkinPatchStatus(std::size_t sens_index) const
{
    yCTrace(CBB, "%zu", sens_index);
    CHECK_SENSOR(sens_index, yarp::dev::MAS_ERROR);
    return deviceMapper.getSensorStatus(&yarp::dev::ISkinPatches::getSkinPatchStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getSkinPatchName(std::size_t sens_index, std::string & name) const
{
    yCTrace(CBB, "%zu", sens_index);
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::ISkinPatches::getSkinPatchName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getSkinPatchMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    yCTrace(CBB, "%zu", sens_index);
    CHECK_SENSOR(sens_index, false);
    return deviceMapper.getSensorOutput(&yarp::dev::ISkinPatches::getSkinPatchMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getSkinPatchSize(std::size_t sens_index) const
{
    yCTrace(CBB, "%zu", sens_index);
    CHECK_SENSOR(sens_index, 0);
    return deviceMapper.getSensorArraySize(&yarp::dev::ISkinPatches::getSkinPatchSize, sens_index);
}

// -----------------------------------------------------------------------------