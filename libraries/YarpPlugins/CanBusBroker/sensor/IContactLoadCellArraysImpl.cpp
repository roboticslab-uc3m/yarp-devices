// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getNrOfContactLoadCellArrays() const
{
    yCTrace(CBB, "");
    return deviceMapper.getConnectedSensors<yarp::dev::IContactLoadCellArrays>();
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getContactLoadCellArrayStatus(std::size_t sens_index) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorStatus(&yarp::dev::IContactLoadCellArrays::getContactLoadCellArrayStatus, sens_index);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getContactLoadCellArrayName(std::size_t sens_index, std::string & name) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IContactLoadCellArrays::getContactLoadCellArrayName, sens_index, name);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getContactLoadCellArrayMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorOutput(&yarp::dev::IContactLoadCellArrays::getContactLoadCellArrayMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getContactLoadCellArraySize(std::size_t sens_index) const
{
    yCTrace(CBB, "%zu", sens_index);
    return deviceMapper.getSensorArraySize(&yarp::dev::IContactLoadCellArrays::getContactLoadCellArraySize, sens_index);
}

// -----------------------------------------------------------------------------
