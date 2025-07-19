#include "PhidgetSpatial.hpp"

// -----------------------------------------------------------------------------

size_t PhidgetSpatial::getNrOfThreeAxisMagnetometers() const
{
    return NUM_SENSORS;
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status PhidgetSpatial::getThreeAxisMagnetometerStatus(size_t sens_index) const
{
    return yarp::dev::MAS_OK;
}

// -----------------------------------------------------------------------------


bool PhidgetSpatial::getThreeAxisMagnetometerName(size_t sens_index, std::string & name) const
{
    CHECK_SENSOR(sens_index);
    name = "magnetometer";
    return true;
}

// -----------------------------------------------------------------------------


bool PhidgetSpatial::getThreeAxisMagnetometerFrameName(size_t sens_index, std::string & frameName) const
{
    return getThreeAxisGyroscopeName(sens_index, frameName);
}

// -----------------------------------------------------------------------------


bool PhidgetSpatial::getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
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

    return true;
}

// -----------------------------------------------------------------------------
