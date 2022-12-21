// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <cmath>

#include <algorithm> // std::clamp

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------------

bool TechnosoftIposExternal::velocityMoveRaw(int j, double sp)
{
    yCITrace(IPOS, id(), "%d %f", j, sp);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_VELOCITY);

    const double maxVel = this->maxVel;

    if (std::abs(sp) > maxVel)
    {
        yCIWarning(IPOS, id(), "Requested speed exceeds maximum velocity (%f)", maxVel);
        sp = std::clamp(sp, -maxVel, maxVel);
    }

    double initialPosition = internalUnitsToDegrees(lastEncoderRead->queryPosition());
    trapTrajectory.configure(syncPeriod, initialPosition, sp, refAcceleration);

    return true;
}

// ----------------------------------------------------------------------------------

bool TechnosoftIposExternal::velocityMoveRaw(const double * sp)
{
    return velocityMoveRaw(0, sp[0]);
}

// ----------------------------------------------------------------------------------

bool TechnosoftIposExternal::velocityMoveRaw(int n_joint, const int * joints, const double * spds)
{
    return velocityMoveRaw(joints[0], spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefVelocityRaw(int joint, double * vel)
{
    yCITrace(IPOS, id(), "%d", joint);
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_VELOCITY);
    *vel = trapTrajectory.queryVelocity();
    return true;
}

// ------------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefVelocitiesRaw(double * vels)
{
    return getRefVelocityRaw(0, &vels[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefVelocitiesRaw(int n_joint, const int * joints, double * vels)
{
    return getRefVelocityRaw(joints[0], &vels[0]);
}

// -----------------------------------------------------------------------------
