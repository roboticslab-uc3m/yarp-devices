// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getNumberOfMotorsRaw(int * number)
{
    std::size_t ax;
    auto ret = getAxes(ax);
    *number = static_cast<int>(ax);
    return ret;
}
#else
bool TechnosoftIposBase::getNumberOfMotorsRaw(int * number)
{
    return getAxes(number);
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getTemperatureRaw(int m, double * val)
#else
bool TechnosoftIposBase::getTemperatureRaw(int m, double * val)
#endif
{
    yCIError(IPOS, id(), "getTemperatureRaw() not supported");
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getTemperaturesRaw(double * vals)
#else
bool TechnosoftIposBase::getTemperaturesRaw(double * vals)
#endif
{
    yCIError(IPOS, id(), "getTemperaturesRaw() not supported");
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getTemperatureLimitRaw(int m, double * temp)
#else
bool TechnosoftIposBase::getTemperatureLimitRaw(int m, double * temp)
#endif
{
    yCIError(IPOS, id(), "getTemperatureLimitRaw() not supported");
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::setTemperatureLimitRaw(int m, double temp)
#else
bool TechnosoftIposBase::setTemperatureLimitRaw(int m, double temp)
#endif
{
    yCIError(IPOS, id(), "setTemperatureLimitRaw() not supported");
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getGearboxRatioRaw(int m, double * val)
#else
bool TechnosoftIposBase::getGearboxRatioRaw(int m, double * val)
#endif
{
    CHECK_JOINT(m);
    *val = tr;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::setGearboxRatioRaw(int m, double val)
#else
bool TechnosoftIposBase::setGearboxRatioRaw(int m, double val)
#endif
{
    CHECK_JOINT(m);
    tr = val;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
