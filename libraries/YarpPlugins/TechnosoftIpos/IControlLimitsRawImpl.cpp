// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------- IControlLimitsRaw Related ------------------------------------

bool roboticslab::TechnosoftIpos::setLimitsRaw(int axis, double min, double max)
{
    CD_INFO("(%d, %f, %f)\n", axis, min, max);
    CHECK_JOINT(axis);
    return setLimitRaw(min, true) & setLimitRaw(max, false);
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

    int32_t data = degreesToInternalUnits(limit);
    return sdoClient->download(name, data, 0x607D, subindex);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getLimitsRaw(int axis, double *min, double *max)
{
    CD_INFO("(%d)\n", axis);
    CHECK_JOINT(axis);
    return getLimitRaw(min, true) & getLimitRaw(max, false);
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

    return sdoClient->upload<int32_t>(name, [=](int32_t * data)
            { *limit = internalUnitsToDegrees(*data); },
            0x607D, subindex);
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setVelLimitsRaw(int axis, double min, double max)
{
    CD_INFO("(%d, %f, %f)\n", axis, min, max);
    CHECK_JOINT(axis);

    maxVel = max;

    if (min != -max)
    {
        CD_WARNING("Minimum value not equal to negative maximum value.\n");
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getVelLimitsRaw(int axis, double *min, double *max)
{
    CD_INFO("(%d)\n", axis);
    CHECK_JOINT(axis);

    *min = -maxVel;
    *max = maxVel;

    return true;
}

// -----------------------------------------------------------------------------
