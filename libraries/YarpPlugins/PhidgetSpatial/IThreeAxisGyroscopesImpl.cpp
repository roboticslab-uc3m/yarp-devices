#include "PhidgetSpatial.hpp"

// -----------------------------------------------------------------------------

size_t PhidgetSpatial::getNrOfThreeAxisGyroscopes() const
{
    return NUM_SENSORS;
}

// -----------------------------------------------------------------------------

yarp::dev::MAS_status PhidgetSpatial::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    return yarp::dev::MAS_OK;
}

// -----------------------------------------------------------------------------


bool PhidgetSpatial::getThreeAxisGyroscopeName(size_t sens_index, std::string & name) const
{
    CHECK_SENSOR(sens_index);
    name = "gyroscope";
    return true;
}

// -----------------------------------------------------------------------------


bool PhidgetSpatial::getThreeAxisGyroscopeFrameName(size_t sens_index, std::string & frameName) const
{
    return getThreeAxisGyroscopeName(sens_index, frameName);
}

// -----------------------------------------------------------------------------


bool PhidgetSpatial::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
{
    CHECK_SENSOR(sens_index);

    {
        std::lock_guard lock(mtx);

        out = {
            angularRate[0],
            angularRate[1],
            angularRate[2]
        };

        timestamp = this->timestamp;
    }

    return true;
}

// -----------------------------------------------------------------------------
