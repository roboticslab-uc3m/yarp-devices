// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getCurrentRaw(int m, double * curr)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    std::int16_t temp = lastCurrentRead;
    *curr = internalUnitsToCurrent(temp);
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getCurrentRangeRaw(int m, double * min, double * max)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);

    return can->sdo()->upload<std::uint16_t>("Current limit", [this, min, max](auto data)
        { *max = internalUnitsToPeakCurrent(data);
          *min = -(*max); },
        0x207F);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::setRefCurrentRaw(int m, double curr)
{
    yCITrace(IPOS, id(), "%d %f", m, curr);
    CHECK_JOINT(m);
    CHECK_MODE(VOCAB_CM_CURRENT);

    const bool state = limitSwitchState.load();

    if (state == INACTIVE || state == POSITIVE && curr <= 0.0 || state == NEGATIVE && curr >= 0.0)
    {
        commandBuffer.accept(curr);
        return true;
    }
    else
    {
        return false;
    }
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getRefCurrentRaw(int m, double * curr)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    CHECK_MODE(VOCAB_CM_CURRENT);
    *curr = commandBuffer.getStoredCommand();
    return true;
}

// -----------------------------------------------------------------------------
