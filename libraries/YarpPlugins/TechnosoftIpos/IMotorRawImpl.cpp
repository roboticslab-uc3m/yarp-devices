// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getNumberOfMotorsRaw(int * number)
{
    return getAxes(number);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getTemperatureRaw(int m, double * val)
{
    CD_ERROR("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getTemperaturesRaw(double * vals)
{
    CD_ERROR("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getTemperatureLimitRaw(int m, double * temp)
{
    CD_ERROR("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setTemperatureLimitRaw(int m, double temp)
{
    CD_ERROR("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getGearboxRatioRaw(int m, double * val)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    *val = vars.tr;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setGearboxRatioRaw(int m, double val)
{
    CD_DEBUG("(%d, %f)\n", m, val);
    CHECK_JOINT(m);
    vars.tr = val;
    return true;
}

// -----------------------------------------------------------------------------
