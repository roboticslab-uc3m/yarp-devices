// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <cmath> // std::abs

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

    const auto state = limitSwitchState.load();

    if (state == INACTIVE || state == POSITIVE && sp <= 0.0 || state == NEGATIVE && sp >= 0.0)
    {
        if (enableCsv)
        {
            commandBuffer.accept(sp);
        }
        else
        {
            trajectory.setTargetVelocity(yarp::os::SystemClock::nowSystem(),
                                         trajectory.queryPosition(), trajectory.queryVelocity(),
                                         sp, refAcceleration);
        }

        return true;
    }
    else
    {
        return false;
    }
}

// ----------------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefVelocityRaw(int joint, double * vel)
{
    yCITrace(IPOS, id(), "%d", joint);
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_VELOCITY);
    *vel = enableCsv ? commandBuffer.getStoredCommand() : trajectory.queryVelocity();
    return true;
}

// ------------------------------------------------------------------------------
