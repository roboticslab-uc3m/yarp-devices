// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <cstring>

#include <algorithm> // std::clamp

#include <yarp/conf/version.h>

using namespace roboticslab;

//--------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::getNumberOfMotorsRaw(int * number)
#else
bool LacqueyFetch::getNumberOfMotorsRaw(int * number)
#endif
{
    *number = 1;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

//--------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::setRefDutyCycleRaw(int m, double ref)
#else
bool LacqueyFetch::setRefDutyCycleRaw(int m, double ref)
#endif
{
    CHECK_JOINT(m);

    ref = std::clamp(ref, -100.0, 100.0);

    const std::size_t len = sizeof(refDutyCycles);
    std::uint8_t msgData[len];
    refDutyCycles = ref;
    std::memcpy(msgData, &refDutyCycles, len);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return send(len, msgData)
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return send(len, msgData);
#endif
}

//--------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::setRefDutyCyclesRaw(const double * refs)
#else
bool LacqueyFetch::setRefDutyCyclesRaw(const double * refs)
#endif
{
    return setRefDutyCycleRaw(0, refs[0]);
}

//--------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::getRefDutyCycleRaw(int m, double * ref)
#else
bool LacqueyFetch::getRefDutyCycleRaw(int m, double * ref)
#endif
{
    CHECK_JOINT(m);
    *ref = refDutyCycles;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

//--------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::getRefDutyCyclesRaw(double * refs)
#else
bool LacqueyFetch::getRefDutyCyclesRaw(double * refs)
#endif
{
    return getRefDutyCycleRaw(0, &refs[0]);
}

//--------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::getDutyCycleRaw(int m, double * val)
#else
bool LacqueyFetch::getDutyCycleRaw(int m, double * val)
#endif
{
    CHECK_JOINT(m);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

//--------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue LacqueyFetch::getDutyCyclesRaw(double * vals)
#else
bool LacqueyFetch::getDutyCyclesRaw(double * vals)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return getDutyCycleRaw(0, &vals[0]);
#else
    return getDutyCycleRaw(0, &vals[0]);
#endif
}

//--------------------------------------------------------------------------------------
