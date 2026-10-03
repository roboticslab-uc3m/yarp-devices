// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
#define CHECK_SENSOR(idx, out) do { std::size_t n; auto ret = getNrOfThreeAxisMagnetometers(n); if (!ret || (idx) < 0 || (idx) > n - 1) return out; } while (0)
#else
#define CHECK_SENSOR(idx, ret) do { int n = getNrOfThreeAxisMagnetometers(); if ((idx) < 0 || (idx) > n - 1) return ret; } while (0)
#endif

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue CanBusBroker::getNrOfThreeAxisMagnetometers(std::size_t & num) const
{
    return deviceMapper.getConnectedSensors<yarp::dev::IThreeAxisMagnetometers>(num);
}
#else
std::size_t CanBusBroker::getNrOfThreeAxisMagnetometers() const
{
    return deviceMapper.getConnectedSensors<yarp::dev::IThreeAxisMagnetometers>();
}
#endif

// -----------------------------------------------------------------------------

yarp::dev::MAS_status CanBusBroker::getThreeAxisMagnetometerStatus(std::size_t sens_index) const
{
    CHECK_SENSOR(sens_index, yarp::dev::MAS_ERROR);
    return deviceMapper.getSensorStatus(&yarp::dev::IThreeAxisMagnetometers::getThreeAxisMagnetometerStatus, sens_index);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue CanBusBroker::getThreeAxisMagnetometerName(std::size_t sens_index, std::string & name) const
#else
bool CanBusBroker::getThreeAxisMagnetometerName(std::size_t sens_index, std::string & name) const
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    CHECK_SENSOR(sens_index, yarp::dev::ReturnValue_error_input_out_of_bounds);
#else
    CHECK_SENSOR(sens_index, false);
#endif
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisMagnetometers::getThreeAxisMagnetometerName, sens_index, name);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue CanBusBroker::getThreeAxisMagnetometerFrameName(std::size_t sens_index, std::string & frameName) const
#else
bool CanBusBroker::getThreeAxisMagnetometerFrameName(std::size_t sens_index, std::string & frameName) const
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    CHECK_SENSOR(sens_index, yarp::dev::ReturnValue_error_input_out_of_bounds);
#else
    CHECK_SENSOR(sens_index, false);
#endif
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisMagnetometers::getThreeAxisMagnetometerFrameName, sens_index, frameName);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue CanBusBroker::getThreeAxisMagnetometerMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#else
bool CanBusBroker::getThreeAxisMagnetometerMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    CHECK_SENSOR(sens_index, yarp::dev::ReturnValue_error_input_out_of_bounds);
#else
    CHECK_SENSOR(sens_index, false);
#endif
    return deviceMapper.getSensorOutput(&yarp::dev::IThreeAxisMagnetometers::getThreeAxisMagnetometerMeasure, sens_index, out, timestamp);
}

// -----------------------------------------------------------------------------
