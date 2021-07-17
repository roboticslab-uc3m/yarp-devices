// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

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
    yCError(IPOS, "getTemperatureRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getTemperaturesRaw(double * vals)
{
    yCError(IPOS, "getTemperaturesRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getTemperatureLimitRaw(int m, double * temp)
{
    yCError(IPOS, "getTemperatureLimitRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setTemperatureLimitRaw(int m, double temp)
{
    yCError(IPOS, "setTemperatureLimitRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getGearboxRatioRaw(int m, double * val)
{
    yCTrace(IPOS, "%d", m);
    CHECK_JOINT(m);
    *val = vars.tr;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setGearboxRatioRaw(int m, double val)
{
    yCTrace(IPOS, "%d %f", m, val);
    CHECK_JOINT(m);
    vars.tr = val;
    return true;
}

// -----------------------------------------------------------------------------
