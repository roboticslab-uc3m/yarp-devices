// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#include <cmath>

#include <algorithm> // std::clamp

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::velocityMoveRaw(int j, double sp)
{
    yCITrace(IPOS, id(), "%d %f", j, sp);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_VELOCITY);

    const auto maxVel = vars.maxVel.load();

    if (std::abs(sp) > maxVel)
    {
        yCIWarning(IPOS, id(), "Requested speed exceeds maximum velocity (%f)", maxVel);
        sp = std::clamp(sp, -maxVel, maxVel);
    }

    // reset halt bit
    if (can->driveStatus()->controlword()[8]
        && !can->driveStatus()->controlword(can->driveStatus()->controlword().reset(8)))
    {
        return false;
    }

    commandBuffer.accept(sp);
    return true;
}

// ----------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::velocityMoveRaw(const double * sp)
{
    return velocityMoveRaw(0, sp[0]);
}

// ----------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::velocityMoveRaw(int n_joint, const int * joints, const double * spds)
{
    return velocityMoveRaw(joints[0], spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRefVelocityRaw(int joint, double * vel)
{
    yCITrace(IPOS, id(), "%d", joint);
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_VELOCITY);
    *vel = commandBuffer.getStoredCommand();
    return true;
}

// ------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRefVelocitiesRaw(double * vels)
{
    return getRefVelocityRaw(0, &vels[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRefVelocitiesRaw(int n_joint, const int * joints, double * vels)
{
    return getRefVelocityRaw(joints[0], &vels[0]);
}

// -----------------------------------------------------------------------------
