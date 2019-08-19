// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>

namespace
{
    // return -1 for negative numbers, +1 for positive numbers, 0 for zero
    // https://stackoverflow.com/a/4609795
    template <typename T>
    inline int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }
}

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

    *curr = data * sgn(tr) * 2.0 * drivePeakCurrent / 65520.0;
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

    *max = 2 * drivePeakCurrent * (32767 - data) / 65520;
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

    int32_t data = curr * sgn(tr) * 65520.0 / (2 * drivePeakCurrent);
    return sdoClient->download<int32_t>("External online reference", data << 16, 0x201C);
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

    *curr = (data >> 16) * sgn(tr) * 2.0 * drivePeakCurrent / 65520.0;
    return true;
}

// -----------------------------------------------------------------------------
