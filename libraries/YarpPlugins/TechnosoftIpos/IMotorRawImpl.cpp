// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/conf/version.h>
#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getNumberOfMotorsRaw(int * number)
{
    return getAxes(number);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getTemperatureRaw(int m, double * val)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCIError(IPOS, id(), "getTemperatureRaw() not supported");
#else
    yCError(IPOS, "getTemperatureRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getTemperaturesRaw(double * vals)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCIError(IPOS, id(), "getTemperaturesRaw() not supported");
#else
    yCError(IPOS, "getTemperaturesRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getTemperatureLimitRaw(int m, double * temp)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCIError(IPOS, id(), "getTemperatureLimitRaw() not supported");
#else
    yCError(IPOS, "getTemperatureLimitRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setTemperatureLimitRaw(int m, double temp)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCIError(IPOS, id(), "setTemperatureLimitRaw() not supported");
#else
    yCError(IPOS, "setTemperatureLimitRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getGearboxRatioRaw(int m, double * val)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d", m);
#else
    yCTrace(IPOS, "%d", m);
#endif
    CHECK_JOINT(m);
    *val = vars.tr;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setGearboxRatioRaw(int m, double val)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d %f", m, val);
#else
    yCTrace(IPOS, "%d %f", m, val);
#endif
    CHECK_JOINT(m);
    vars.tr = val;
    return true;
}

// -----------------------------------------------------------------------------
