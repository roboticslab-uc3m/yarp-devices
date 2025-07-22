#include "PhidgetSpatial.hpp"

// -----------------------------------------------------------------------------

size_t PhidgetSpatial::getNrOfThreeAxisLinearAccelerometers() const
{
    return NUM_SENSORS;
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status PhidgetSpatial::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    return yarp::dev::MAS_OK;
}

// -----------------------------------------------------------------------------


bool PhidgetSpatial::getThreeAxisLinearAccelerometerName(size_t sens_index, std::string & name) const
{
    CHECK_SENSOR(sens_index);
    name = "accelerometer";
    return true;
}

// -----------------------------------------------------------------------------


bool PhidgetSpatial::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string & frameName) const
{
    return getThreeAxisGyroscopeName(sens_index, frameName);
}

// -----------------------------------------------------------------------------


bool PhidgetSpatial::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
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

    return true;
}

// -----------------------------------------------------------------------------
