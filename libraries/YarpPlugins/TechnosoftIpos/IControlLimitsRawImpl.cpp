// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::setLimitsRaw(int axis, double min, double max)
{
    yCITrace(IPOS, id(), "%d %f %f", axis, min, max);
    CHECK_JOINT(axis);

    bool okMin = false;
    bool okMax = false;

    if (setLimitRaw(min, true))
    {
        this->min = min;
        okMin = true;
    }

    if (setLimitRaw(max, false))
    {
        this->max = max;
        okMax = true;
    }

    return okMin && okMax;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::setLimitRaw(double limit, bool isMin)
{
    std::string name = "Software position limit: ";
    std::uint8_t subindex;

    if (isMin ^ reverse)
    {
        name += "minimal position limit";
        subindex = 0x01;
    }
    else
    {
        name += "maximal position limit";
        subindex = 0x02;
    }

    std::int32_t data = degreesToInternalUnits(limit);
    return can->sdo()->download(name, data, 0x607D, subindex);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getLimitsRaw(int axis, double * min, double * max)
{
    yCITrace(IPOS, id(), "%d", axis);
    CHECK_JOINT(axis);

    if (actualControlMode == VOCAB_CM_NOT_CONFIGURED)
    {
        *min = this->min;
        *max = this->max;
        return true;
    }

    return getLimitRaw(min, true) & getLimitRaw(max, false);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getLimitRaw(double * limit, bool isMin)
{
    std::string name = "Software position limit: ";
    std::uint8_t subindex;

    if (isMin ^ reverse)
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
        { *limit = internalUnitsToDegrees(data); },
        0x607D, subindex);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::setVelLimitsRaw(int axis, double min, double max)
{
    yCITrace(IPOS, id(), "%d %f %f", axis, min, max);
    CHECK_JOINT(axis);

    maxVel = max;

    if (min != -max)
    {
        yCIWarning(IPOS, id()) << "Minimum value not equal to negative maximum value";
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getVelLimitsRaw(int axis, double * min, double * max)
{
    yCITrace(IPOS, id(), "%d", axis);
    CHECK_JOINT(axis);

    *min = -maxVel;
    *max = maxVel;

    return true;
}

// -----------------------------------------------------------------------------
