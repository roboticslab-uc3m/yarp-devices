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
    std::int16_t temp = vars.lastCurrentRead;
    *curr = vars.internalUnitsToCurrent(temp);
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getCurrentsRaw(double * currs)
{
    return getCurrentRaw(0, &currs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getCurrentRangeRaw(int m, double * min, double * max)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);

    return can->sdo()->upload<std::uint16_t>("Current limit", [this, min, max](auto data)
        { *max = vars.internalUnitsToPeakCurrent(data);
          *min = -(*max); },
        0x207F);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getCurrentRangesRaw(double * min, double * max)
{
    return getCurrentRangeRaw(0, min, max);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::setRefCurrentRaw(int m, double curr)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    CHECK_MODE(VOCAB_CM_CURRENT);
    commandBuffer.accept(curr);
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::setRefCurrentsRaw(const double * currs)
{
    return setRefCurrentRaw(0, currs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::setRefCurrentsRaw(int n_motor, const int * motors, const double * currs)
{
    return setRefCurrentRaw(motors[0], currs[0]);
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

bool TechnosoftIposBase::getRefCurrentsRaw(double * currs)
{
    return getRefCurrentRaw(0, &currs[0]);
}

// -----------------------------------------------------------------------------
