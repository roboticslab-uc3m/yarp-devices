// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <cmath>

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::positionMoveRaw(int j, double ref)
{
    yCITrace(IPOS, id(), "%d %f", j, ref);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    const auto state = limitSwitchState.load();

    if (state == INACTIVE || state == POSITIVE && ref <= max || state == NEGATIVE && ref >= min)
    {
        trajectory.setTargetPosition(yarp::os::SystemClock::nowSystem(),
                                     trajectory.queryPosition(), trajectory.queryVelocity(),
                                     ref, refSpeed, refAcceleration);

        return true;
    }
    else
    {
        return false;
    }
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::relativeMoveRaw(int j, double delta)
{
    yCITrace(IPOS, id(), "%d %f", j, delta);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    const auto state = limitSwitchState.load();

    if (state == INACTIVE || state == POSITIVE && delta <= 0.0 || state == NEGATIVE && delta >= 0.0)
    {
        trajectory.setTargetPosition(yarp::os::SystemClock::nowSystem(),
                                     trajectory.queryPosition(), trajectory.queryVelocity(),
                                     trajectory.queryPosition() + delta, refSpeed, refAcceleration);

        return true;
    }
    else
    {
        return false;
    }
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::checkMotionDoneRaw(int j, bool * flag)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    *flag = !trajectory.isActive();
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::setRefSpeedRaw(int j, double sp)
{
    yCITrace(IPOS, id(), "%d %f", j, sp);
    CHECK_JOINT(j);

    if (sp < 0.0)
    {
        yCIError(IPOS, id()) << "Illegal reference speed provided:" << sp;
        return false;
    }
    else if (sp == 0.0)
    {
        yCIWarning(IPOS, id()) << "Reference speed is zero, i.e. no motion will be performed";
    }
    else if (sp > maxVel)
    {
        yCIWarning(IPOS, id()) << "Reference speed exceeds maximum velocity:" << sp << ">" << maxVel.load() << "(will be clipped)";
        sp = maxVel;
    }

    if (trajectory.isActive())
    {
        trajectory.setTargetPosition(yarp::os::SystemClock::nowSystem(),
                                     trajectory.queryPosition(), trajectory.queryVelocity(),
                                     trajectory.getTargetPosition(), sp, refAcceleration);
    }

    refSpeed = sp;
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::setRefAccelerationRaw(int j, double acc)
{
    yCITrace(IPOS, id(), "%d %f", j, acc);
    CHECK_JOINT(j);

    if (acc <= 0.0)
    {
        yCIError(IPOS, id()) << "Illegal reference acceleration provided:" << acc;
        return false;
    }

    if (trajectory.isActive())
    {
        trajectory.setTargetPosition(yarp::os::SystemClock::nowSystem(),
                                     trajectory.queryPosition(), trajectory.queryVelocity(),
                                     trajectory.getTargetPosition(), refSpeed, acc);
    }

    refAcceleration = acc;
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefSpeedRaw(int j, double * ref)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    *ref = refSpeed;
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefAccelerationRaw(int j, double * acc)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    *acc = refAcceleration;
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::stopRaw(int j)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    if (actualControlMode != VOCAB_CM_POSITION && actualControlMode != VOCAB_CM_VELOCITY)
    {
        return false;
    }

    if (actualControlMode == VOCAB_CM_VELOCITY && enableCsv)
    {
        commandBuffer.accept(0.0);
    }
    else if (trajectory.isActive())
    {
        trajectory.setTargetVelocity(yarp::os::SystemClock::nowSystem(),
                                     trajectory.queryPosition(), trajectory.queryVelocity(),
                                     0.0, refAcceleration);
    }

    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::getTargetPositionRaw(int joint, double * ref)
{
    yCITrace(IPOS, id(), "%d", joint);
    CHECK_JOINT(joint);
    *ref = trajectory.getTargetPosition();
    return true;
}

// --------------------------------------------------------------------------------
