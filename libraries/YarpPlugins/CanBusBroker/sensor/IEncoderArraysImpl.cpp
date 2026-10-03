// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
#define CHECK_SENSOR(idx, out) do { std::size_t n; auto ret = getNrOfEncoderArrays(n); if (!ret || (idx) < 0 || (idx) > n - 1) return out; } while (0)
#else
#define CHECK_SENSOR(idx, ret) do { int n = getNrOfEncoderArrays(); if ((idx) < 0 || (idx) > n - 1) return ret; } while (0)
#endif

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue CanBusBroker::getNrOfEncoderArrays(std::size_t & num) const
{
    return deviceMapper.getConnectedSensors<yarp::dev::IEncoderArrays>(num);
}
#else
std::size_t CanBusBroker::getNrOfEncoderArrays() const
{
    return deviceMapper.getConnectedSensors<yarp::dev::IEncoderArrays>();
}
#endif

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getEncoderArrayStatus(std::size_t sens_index) const
{
    CHECK_SENSOR(sens_index, yarp::dev::MAS_ERROR);
    return deviceMapper.getSensorStatus(&yarp::dev::IEncoderArrays::getEncoderArrayStatus, sens_index);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue CanBusBroker::getEncoderArrayName(std::size_t sens_index, std::string & name) const
#else
bool CanBusBroker::getEncoderArrayName(std::size_t sens_index, std::string & name) const
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    CHECK_SENSOR(sens_index, yarp::dev::ReturnValue_error_input_out_of_bounds);
#else
    CHECK_SENSOR(sens_index, false);
#endif
    return deviceMapper.getSensorOutput(&yarp::dev::IEncoderArrays::getEncoderArrayName, sens_index, name);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue CanBusBroker::getEncoderArrayMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#else
bool CanBusBroker::getEncoderArrayMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    CHECK_SENSOR(sens_index, yarp::dev::ReturnValue_error_input_out_of_bounds);
#else
    CHECK_SENSOR(sens_index, false);
#endif
    return deviceMapper.getSensorOutput(&yarp::dev::IEncoderArrays::getEncoderArrayMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------

std::size_t CanBusBroker::getEncoderArraySize(std::size_t sens_index) const
{
    CHECK_SENSOR(sens_index, 0);
    return deviceMapper.getSensorArraySize(&yarp::dev::IEncoderArrays::getEncoderArraySize, sens_index);
}

// -----------------------------------------------------------------------------
