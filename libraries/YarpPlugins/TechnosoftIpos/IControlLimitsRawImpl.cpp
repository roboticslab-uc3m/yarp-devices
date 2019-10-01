// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setLimitsRaw(int axis, double min, double max)
{
    CD_DEBUG("(%d, %f, %f)\n", axis, min, max);
    CHECK_JOINT(axis);
    return setLimitRaw(min, true) & setLimitRaw(max, false);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setLimitRaw(double limit, bool isMin)
{
    std::string name = "Software position limit: ";
    std::uint8_t subindex;

    if (isMin ^ vars.tr < 0)
    {
        name += "minimal position limit";
        subindex = 0x01;
    }
    else
    {
        name += "maximal position limit";
        subindex = 0x02;
    }

    std::int32_t data = vars.degreesToInternalUnits(limit);
    return can->sdo()->download(name, data, 0x607D, subindex);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getLimitsRaw(int axis, double * min, double * max)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);
    return getLimitRaw(min, true) & getLimitRaw(max, false);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getLimitRaw(double * limit, bool isMin)
{
    std::string name = "Software position limit: ";
    std::uint8_t subindex;

    if (vars.tr >= 0)
    {
        name += "minimal position limit";
        subindex = 0x01;
    }
    else
    {
        name += "maximal position limit";
        subindex = 0x02;
    }

    return can->sdo()->upload<std::int32_t>(name, [=](std::int32_t * data)
            { *limit = vars.internalUnitsToDegrees(*data); },
            0x607D, subindex);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setVelLimitsRaw(int axis, double min, double max)
{
    CD_DEBUG("(%d, %f, %f)\n", axis, min, max);
    CHECK_JOINT(axis);

    vars.maxVel = max;

    if (min != -max)
    {
        CD_WARNING("Minimum value not equal to negative maximum value.\n");
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getVelLimitsRaw(int axis, double * min, double * max)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);

    *min = -vars.maxVel;
    *max = vars.maxVel;

    return true;
}

// -----------------------------------------------------------------------------
