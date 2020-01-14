// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getCurrentRaw(int m, double * curr)
{
    //CD_DEBUG("(%d)\n", m); // too verbose in controlboardwrapper2 stream
    CHECK_JOINT(m);
    std::int16_t temp = vars.lastCurrentRead;
    *curr = vars.internalUnitsToCurrent(temp);
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getCurrentsRaw(double * currs)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    return getCurrentRaw(0, &currs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getCurrentRangeRaw(int m, double * min, double * max)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);

    return can->sdo()->upload<std::uint16_t>("Current limit", [&](std::uint16_t data)
            { *max = vars.internalUnitsToPeakCurrent(data);
              *min = -(*max); },
            0x207F);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getCurrentRangesRaw(double * min, double * max)
{
    CD_DEBUG("\n");
    return getCurrentRangeRaw(0, min, max);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRefCurrentRaw(int m, double curr)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    CHECK_MODE(VOCAB_CM_CURRENT);
    vars.synchronousCommandTarget = curr;
    return quitHaltState(VOCAB_CM_CURRENT);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRefCurrentsRaw(const double * currs)
{
    CD_DEBUG("\n");
    return setRefCurrentRaw(0, currs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRefCurrentsRaw(int n_motor, const int * motors, const double * currs)
{
    CD_DEBUG("\n");
    return setRefCurrentRaw(motors[0], currs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefCurrentRaw(int m, double * curr)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    CHECK_MODE(VOCAB_CM_CURRENT);
    *curr = vars.synchronousCommandTarget;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefCurrentsRaw(double * currs)
{
    CD_DEBUG("\n");
    return getRefCurrentRaw(0, &currs[0]);
}

// -----------------------------------------------------------------------------
