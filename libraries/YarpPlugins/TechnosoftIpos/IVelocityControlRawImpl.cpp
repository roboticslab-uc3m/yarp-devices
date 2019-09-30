// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------------

bool TechnosoftIpos::velocityMoveRaw(int j, double sp)
{
    CD_DEBUG("(%d, %f)\n", j, sp);
    CHECK_JOINT(j);

    if (sp > vars.maxVel)
    {
        CD_WARNING("Requested speed exceeds maximum velocity (%f).\n", vars.maxVel);
        return false;
    }

    double value = vars.degreesToInternalUnits(sp, 1);

    int16_t dataInt;
    uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    int32_t data = (dataInt << 16) + dataFrac;
    return can->sdo()->download("Target velocity", data, 0x60FF);
}

// ----------------------------------------------------------------------------------

bool TechnosoftIpos::velocityMoveRaw(const double * sp)
{
    CD_DEBUG("\n");
    return velocityMoveRaw(0, sp[0]);
}

// ----------------------------------------------------------------------------------

bool TechnosoftIpos::velocityMoveRaw(int n_joint, const int * joints, const double * spds)
{
    CD_DEBUG("\n");
    return velocityMoveRaw(joints[0], spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefVelocityRaw(int joint, double * vel)
{
    CD_DEBUG("(%d)\n",joint);
    CHECK_JOINT(joint);

    return can->sdo()->upload<int32_t>("Target velocity", [=](int32_t * data)
            {
                int16_t dataInt = *data >> 16;
                uint16_t dataFrac = *data & 0xFFFF;
                double value = CanUtils::decodeFixedPoint(dataInt, dataFrac);
                *vel = vars.internalUnitsToDegrees(value, 1);
            },
            0x60FF);
}

// ------------------------------------------------------------------------------

bool TechnosoftIpos::getRefVelocitiesRaw(double * vels)
{
    CD_DEBUG("\n");
    return getRefVelocityRaw(0, &vels[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefVelocitiesRaw(int n_joint, const int * joints, double * vels)
{
    CD_DEBUG("\n");
    return getRefVelocityRaw(joints[0], &vels[0]);
}

// -----------------------------------------------------------------------------
