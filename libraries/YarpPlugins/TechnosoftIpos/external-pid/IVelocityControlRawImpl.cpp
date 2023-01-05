// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <cmath>

#include <algorithm> // std::clamp

#include <yarp/os/Log.h>
#include <yarp/os/SystemClock.h>

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
    double initialVelocity = internalUnitsToDegrees(lastEncoderRead->querySpeed(), 1);

    trapTrajectory.setTargetVelocity(yarp::os::SystemClock::nowSystem(), initialPosition, initialVelocity, sp, refAcceleration);
    return true;
}

// ----------------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefVelocityRaw(int joint, double * vel)
{
    yCITrace(IPOS, id(), "%d", joint);
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_VELOCITY);
    *vel = trapTrajectory.queryVelocity();
    return true;
}

// ------------------------------------------------------------------------------
