// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cmath>

#include <ColorDebug.h>

using namespace roboticslab;

// ----------------------------------------------------------------------------------

bool TechnosoftIpos::velocityMoveRaw(int j, double sp)
{
    CD_DEBUG("(%d, %f)\n", j, sp);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_VELOCITY);

    if (std::abs(sp) > vars.maxVel)
    {
        CD_WARNING("Requested speed exceeds maximum velocity (%f).\n", vars.maxVel.load());
        sp = std::min<double>(vars.maxVel, std::max<double>(-vars.maxVel, sp));
    }

    // reset halt bit
    if (can->driveStatus()->controlword()[8]
        && !can->driveStatus()->controlword(can->driveStatus()->controlword().reset(8)))
    {
        return false;
    }

    vars.synchronousCommandTarget = sp;
    return true;
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
    CHECK_MODE(VOCAB_CM_VELOCITY);
    *vel = vars.synchronousCommandTarget;
    return true;
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
