// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getNumberOfMotorsRaw(int * number)
{
    return getAxes(number);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getTemperatureRaw(int m, double * val)
{}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getTemperaturesRaw(double * vals)
{}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getTemperatureLimitRaw(int m, double * temp)
{}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setTemperatureLimitRaw(int m, double temp)
{}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getGearboxRatioRaw(int m, double * val)
{}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setGearboxRatioRaw(int m, const double val)
{}

// -----------------------------------------------------------------------------
