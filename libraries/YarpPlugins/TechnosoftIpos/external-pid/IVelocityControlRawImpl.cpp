// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <cmath> // std::abs

#include <algorithm> // std::clamp

#include <yarp/os/Log.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::velocityMoveRaw(int j, double sp)
#else
bool TechnosoftIposExternal::velocityMoveRaw(int j, double sp)
#endif
{
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

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_ok;
#else
        return true;
#endif
    }
    else
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }
}

// ----------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getTargetVelocityRaw(int joint, double * vel)
#else
bool TechnosoftIposExternal::getRefVelocityRaw(int joint, double * vel)
#endif
{
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_VELOCITY);
    *vel = enableCsv ? commandBuffer.getStoredCommand() : trajectory.queryVelocity();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// ------------------------------------------------------------------------------
