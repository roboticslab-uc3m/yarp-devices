// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getCurrentRaw(int m, double * curr)
{
    yCTrace(IPOS, "%d", m);
    CHECK_JOINT(m);
    std::int16_t temp = vars.lastCurrentRead;
    *curr = vars.internalUnitsToCurrent(temp);
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getCurrentsRaw(double * currs)
{
    return getCurrentRaw(0, &currs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getCurrentRangeRaw(int m, double * min, double * max)
{
    yCTrace(IPOS, "%d", m);
    CHECK_JOINT(m);

    return can->sdo()->upload<std::uint16_t>("Current limit", [this, min, max](auto data)
        { *max = vars.internalUnitsToPeakCurrent(data);
          *min = -(*max); },
        0x207F);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getCurrentRangesRaw(double * min, double * max)
{
    return getCurrentRangeRaw(0, min, max);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRefCurrentRaw(int m, double curr)
{
    yCTrace(IPOS, "%d", m);
    CHECK_JOINT(m);
    CHECK_MODE(VOCAB_CM_CURRENT);
    vars.synchronousCommandTarget = curr;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRefCurrentsRaw(const double * currs)
{
    return setRefCurrentRaw(0, currs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRefCurrentsRaw(int n_motor, const int * motors, const double * currs)
{
    return setRefCurrentRaw(motors[0], currs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefCurrentRaw(int m, double * curr)
{
    yCTrace(IPOS, "%d", m);
    CHECK_JOINT(m);
    CHECK_MODE(VOCAB_CM_CURRENT);
    *curr = vars.synchronousCommandTarget;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefCurrentsRaw(double * currs)
{
    return getRefCurrentRaw(0, &currs[0]);
}

// -----------------------------------------------------------------------------
