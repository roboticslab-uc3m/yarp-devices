// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------- IControlLimitsRaw Related ------------------------------------

bool roboticslab::TechnosoftIpos::setLimitsRaw(int axis, double min, double max)
{
    CD_INFO("(%d,%f,%f)\n",axis,min,max);

    //-- Check index within range
    if ( axis != 0 ) return false;

    bool ok = true;
    ok &= setLimitRaw(min, true);
    ok &= setLimitRaw(max, false);

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setLimitRaw(double limit, bool isMin)
{
    std::string name = "Software position limit: ";
    uint8_t subindex;

    if (isMin ^ tr < 0)
    {
        name += "minimal position limit";
        subindex = 0x01;
    }
    else
    {
        name += "maximal position limit";
        subindex = 0x02;
    }

    int32_t data = applyInternalUnits(limit);
    return sdoClient->download(name, data, 0x607D, subindex);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getLimitsRaw(int axis, double *min, double *max)
{
    CD_INFO("(%d)\n",axis);

    //-- Check index within range
    if( axis != 0 ) return false;

    bool ok = true;
    ok &= getLimitRaw(min, true);
    ok &= getLimitRaw(max, false);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getLimitRaw(double * limit, bool isMin)
{
    std::string name = "Software position limit: ";
    uint8_t subindex;

    if (tr >= 0)
    {
        name += "minimal position limit";
        subindex = 0x01;
    }
    else
    {
        name += "maximal position limit";
        subindex = 0x02;
    }

    int32_t data;

    if (!sdoClient->upload(name, &data, 0x607D, subindex))
    {
        return false;
    }

    *limit = parseInternalUnits(data);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setVelLimitsRaw(int axis, double min, double max)
{
    CD_INFO("(%d,%f,%f)\n",axis,min,max);

    //-- Check index within range
    if ( axis != 0 ) return false;

    //-- Update the limits that have been locally stored.
    this->maxVel = max;

    if (min != -max)
    {
        CD_WARNING("Minimum value not equal to negative maximum value.\n");
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getVelLimitsRaw(int axis, double *min, double *max)
{
    CD_INFO("(%d)\n",axis);

    //-- Check index within range
    if( axis != 0 ) return false;

    //-- Get the limits that have been locally stored.
    *min = -this->maxVel;
    *max = this->maxVel;

    return true;
}

// -----------------------------------------------------------------------------
