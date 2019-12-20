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

    if (can->driveStatus()->controlword()[8])
    {
        if (!can->driveStatus()->controlword(can->driveStatus()->controlword().reset(8)))
        {
            CD_ERROR("Unable to reset halt bit.\n");
            return false;
        }
    }

    std::int32_t data = vars.currentToInternalUnits(curr) << 16;
    return can->sdo()->download("External online reference", data, 0x201C);
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

    return can->sdo()->upload<std::int32_t>("External online reference", [&](std::int32_t data)
            { *curr = vars.internalUnitsToCurrent(data >> 16); },
            0x201C);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefCurrentsRaw(double * currs)
{
    CD_DEBUG("\n");
    return getRefCurrentRaw(0, &currs[0]);
}

// -----------------------------------------------------------------------------
