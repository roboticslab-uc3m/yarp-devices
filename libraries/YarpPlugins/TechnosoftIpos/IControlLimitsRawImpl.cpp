// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setLimitsRaw(int axis, double min, double max)
{
    yCITrace(IPOS, id(), "%d %f %f", axis, min, max);
    CHECK_JOINT(axis);

    bool okMin = false;
    bool okMax = false;

    if (setLimitRaw(min, true))
    {
        vars.min = min;
        okMin = true;
    }

    if (setLimitRaw(max, false))
    {
        vars.max = max;
        okMax = true;
    }

    return okMin && okMax;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setLimitRaw(double limit, bool isMin)
{
    std::string name = "Software position limit: ";
    std::uint8_t subindex;

    if (isMin ^ vars.reverse)
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
    yCITrace(IPOS, id(), "%d", axis);
    CHECK_JOINT(axis);

    if (vars.actualControlMode == VOCAB_CM_NOT_CONFIGURED)
    {
        *min = vars.min;
        *max = vars.max;
        return true;
    }

    return getLimitRaw(min, true) & getLimitRaw(max, false);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getLimitRaw(double * limit, bool isMin)
{
    std::string name = "Software position limit: ";
    std::uint8_t subindex;

    if (isMin ^ vars.reverse)
    {
        name += "minimal position limit";
        subindex = 0x01;
    }
    else
    {
        name += "maximal position limit";
        subindex = 0x02;
    }

    return can->sdo()->upload<std::int32_t>(name, [this, limit](auto data)
        { *limit = vars.internalUnitsToDegrees(data); },
        0x607D, subindex);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setVelLimitsRaw(int axis, double min, double max)
{
    yCITrace(IPOS, id(), "%d %f %f", axis, min, max);
    CHECK_JOINT(axis);

    vars.maxVel = max;

    if (min != -max)
    {
        yCIWarning(IPOS, id()) << "Minimum value not equal to negative maximum value";
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getVelLimitsRaw(int axis, double * min, double * max)
{
    yCITrace(IPOS, id(), "%d", axis);
    CHECK_JOINT(axis);

    *min = -vars.maxVel;
    *max = vars.maxVel;

    return true;
}

// -----------------------------------------------------------------------------
