// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
#define CHECK_SENSOR(idx, out) do { std::size_t n; auto ret = getNrOfContactLoadCellArrays(n); if (!ret || (idx) < 0 || (idx) > n - 1) return out; } while (0)
#else
#define CHECK_SENSOR(idx, ret) do { int n = getNrOfContactLoadCellArrays(); if ((idx) < 0 || (idx) > n - 1) return ret; } while (0)
#endif

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue CanBusBroker::getNrOfContactLoadCellArrays(std::size_t & num) const
{
    return deviceMapper.getConnectedSensors<yarp::dev::IContactLoadCellArrays>(num);
}
#else
std::size_t CanBusBroker::getNrOfContactLoadCellArrays() const
{
    return deviceMapper.getConnectedSensors<yarp::dev::IContactLoadCellArrays>();
}
#endif

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getContactLoadCellArrayStatus(std::size_t sens_index) const
{
    CHECK_SENSOR(sens_index, yarp::dev::MAS_ERROR);
    return deviceMapper.getSensorStatus(&yarp::dev::IContactLoadCellArrays::getContactLoadCellArrayStatus, sens_index);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue CanBusBroker::getContactLoadCellArrayName(std::size_t sens_index, std::string & name) const
#else
bool CanBusBroker::getContactLoadCellArrayName(std::size_t sens_index, std::string & name) const
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    CHECK_SENSOR(sens_index, yarp::dev::ReturnValue_error_input_out_of_bounds);
#else
    CHECK_SENSOR(sens_index, false);
#endif
    return deviceMapper.getSensorOutput(&yarp::dev::IContactLoadCellArrays::getContactLoadCellArrayName, sens_index, name);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue CanBusBroker::getContactLoadCellArrayMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#else
bool CanBusBroker::getContactLoadCellArrayMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    CHECK_SENSOR(sens_index, yarp::dev::ReturnValue_error_input_out_of_bounds);
#else
    CHECK_SENSOR(sens_index, false);
#endif
    return deviceMapper.getSensorOutput(&yarp::dev::IContactLoadCellArrays::getContactLoadCellArrayMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getContactLoadCellArraySize(std::size_t sens_index) const
{
    CHECK_SENSOR(sens_index, 0);
    return deviceMapper.getSensorArraySize(&yarp::dev::IContactLoadCellArrays::getContactLoadCellArraySize, sens_index);
}

// -----------------------------------------------------------------------------
