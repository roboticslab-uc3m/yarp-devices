// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>

// ------------------- ICurrentControlRaw Related ------------------------------------

bool roboticslab::TechnosoftIpos::getNumberOfMotorsRaw(int *number)
{
    CD_DEBUG("\n");
    return getAxes(number);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getCurrentRaw(int m, double *curr)
{
    CD_DEBUG("(%d)\n", m);

    //-- Check index within range
    if (m != 0) return false;

    int16_t data;

    if (!sdoClient->upload("Current actual value", &data, 0x207E))
    {
        return false;
    }

    *curr = internalUnitsToCurrent(data);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getCurrentsRaw(double *currs)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getCurrentRangeRaw(int m, double *min, double *max)
{
    CD_DEBUG("(%d)\n", m);

    //-- Check index within range
    if (m != 0) return false;

    uint16_t data;

    if (!sdoClient->upload("Current limit", &data, 0x207F))
    {
        return false;
    }

    *max = internalUnitsToPeakCurrent(data);
    *min = -(*max);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getCurrentRangesRaw(double *min, double *max)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefCurrentsRaw(const double *currs)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefCurrentRaw(int m, double curr)
{
    CD_DEBUG("(%d)\n", m);

    //-- Check index within range
    if (m != 0) return false;

    int32_t data = currentToInternalUnits(curr) << 16;
    return sdoClient->download("External online reference", data, 0x201C);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefCurrentsRaw(const int n_motor, const int *motors, const double *currs)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefCurrentsRaw(double *currs)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefCurrentRaw(int m, double *curr)
{
    CD_DEBUG("(%d)\n", m);

    //-- Check index within range
    if (m != 0) return false;

    int32_t data;

    if (!sdoClient->upload("External online reference", &data, 0x201C))
    {
        return false;
    }

    *curr = internalUnitsToCurrent(data >> 16);
    return true;
}

// -----------------------------------------------------------------------------
