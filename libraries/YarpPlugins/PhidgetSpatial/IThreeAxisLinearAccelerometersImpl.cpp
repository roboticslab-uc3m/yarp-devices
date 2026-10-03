#include "PhidgetSpatial.hpp"

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue PhidgetSpatial::getNrOfThreeAxisLinearAccelerometers(std::size_t & num) const
{
    num = NUM_SENSORS;
    return yarp::dev::ReturnValue_ok;
}
#else
std::size_t PhidgetSpatial::getNrOfThreeAxisLinearAccelerometers() const
{
    return NUM_SENSORS;
}
#endif

// -----------------------------------------------------------------------------

yarp::dev::MAS_status PhidgetSpatial::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    return yarp::dev::MAS_OK;
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue PhidgetSpatial::getThreeAxisLinearAccelerometerName(size_t sens_index, std::string & name) const
#else
bool PhidgetSpatial::getThreeAxisLinearAccelerometerName(size_t sens_index, std::string & name) const
#endif
{
    CHECK_SENSOR(sens_index);
    name = "accelerometer";
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue PhidgetSpatial::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string & frameName) const
#else
bool PhidgetSpatial::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string & frameName) const
#endif
{
    return getThreeAxisLinearAccelerometerName(sens_index, frameName);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue PhidgetSpatial::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#else
bool PhidgetSpatial::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#endif
{
    CHECK_SENSOR(sens_index);

    {
        std::lock_guard lock(mtx);

        out = {
            acceleration[0],
            acceleration[1],
            acceleration[2]
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
