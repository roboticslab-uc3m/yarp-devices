#include "PhidgetSpatial.hpp"

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue PhidgetSpatial::getNrOfThreeAxisGyroscopes(std::size_t & num) const
{
    num = NUM_SENSORS;
    return yarp::dev::ReturnValue_ok;
}
#else
std::size_t PhidgetSpatial::getNrOfThreeAxisGyroscopes() const
{
    return NUM_SENSORS;
}
#endif

// -----------------------------------------------------------------------------

yarp::dev::MAS_status PhidgetSpatial::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    return yarp::dev::MAS_OK;
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue PhidgetSpatial::getThreeAxisGyroscopeName(size_t sens_index, std::string & name) const
#else
bool PhidgetSpatial::getThreeAxisGyroscopeName(size_t sens_index, std::string & name) const
#endif
{
    CHECK_SENSOR(sens_index);
    name = "gyroscope";
#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue PhidgetSpatial::getThreeAxisGyroscopeFrameName(size_t sens_index, std::string & frameName) const
#else
bool PhidgetSpatial::getThreeAxisGyroscopeFrameName(size_t sens_index, std::string & frameName) const
#endif
{
    return getThreeAxisGyroscopeName(sens_index, frameName);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
yarp::dev::ReturnValue PhidgetSpatial::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#else
bool PhidgetSpatial::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector & out, double & timestamp) const
#endif
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

#if YARP_VERSION_COMPARE(>=, 4, 1, 0) || defined(YARP_NEXT)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
