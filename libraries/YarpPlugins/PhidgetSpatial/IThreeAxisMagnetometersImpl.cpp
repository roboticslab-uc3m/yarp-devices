#include "PhidgetSpatial.hpp"

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue PhidgetSpatial::getNrOfThreeAxisMagnetometers(std::size_t & num) const
{
    num = NUM_SENSORS;
    return yarp::dev::ReturnValue_ok;
}
#else
std::size_t PhidgetSpatial::getNrOfThreeAxisMagnetometers() const
{
    return NUM_SENSORS;
}
#endif

// -----------------------------------------------------------------------------

yarp::dev::MAS_status PhidgetSpatial::getThreeAxisMagnetometerStatus(size_t sens_index) const
{
    return yarp::dev::MAS_OK;
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue PhidgetSpatial::getThreeAxisMagnetometerName(size_t sens_index, std::string & name) const
#else
bool PhidgetSpatial::getThreeAxisMagnetometerName(size_t sens_index, std::string & name) const
#endif
{
    CHECK_SENSOR(sens_index);
    name = "magnetometer";
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue PhidgetSpatial::getThreeAxisMagnetometerFrameName(size_t sens_index, std::string & frameName) const
#else
bool PhidgetSpatial::getThreeAxisMagnetometerFrameName(size_t sens_index, std::string & frameName) const
#endif
{
    return getThreeAxisMagnetometerName(sens_index, frameName);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue PhidgetSpatial::getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#else
bool PhidgetSpatial::getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#endif
{
    CHECK_SENSOR(sens_index);

    {
        std::lock_guard lock(mtx);

        out = {
            magneticField[0],
            magneticField[1],
            magneticField[2]
        };

        timestamp = this->timestamp;
    }

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
