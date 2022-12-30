// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <cmath>

#include <yarp/os/LogStream.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::positionMoveRaw(int j, double ref)
{
    yCITrace(IPOS, id(), "%d %f", j, ref);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    double initialPosition = internalUnitsToDegrees(lastEncoderRead->queryPosition());
    trapTrajectory.configure(initialPosition, ref, refSpeed, refAcceleration);

    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::relativeMoveRaw(int j, double delta)
{
    yCITrace(IPOS, id(), "%d %f", j, delta);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    double initialPosition = internalUnitsToDegrees(lastEncoderRead->queryPosition());
    trapTrajectory.configure(initialPosition, initialPosition + delta, refSpeed, refAcceleration);

    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::checkMotionDoneRaw(int j, bool * flag)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    *flag = !trapTrajectory.isActive();
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::setRefSpeedRaw(int j, double sp)
{
    yCITrace(IPOS, id(), "%d %f", j, sp);
    CHECK_JOINT(j);

    if (sp <= 0.0)
    {
        yCIWarning(IPOS, id()) << "Illegal negative speed provided:" << sp;
        return false;
    }
    else if (sp > maxVel)
    {
        yCIWarning(IPOS, id()) << "Reference speed exceeds maximum velocity, i.e." << maxVel.load();
        return false;
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
        yCIWarning(IPOS, id()) << "Illegal negative acceleration provided:" << acc;
        return false;
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

    trapTrajectory.reset(internalUnitsToDegrees(lastEncoderRead->queryPosition()));
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::getTargetPositionRaw(int joint, double * ref)
{
    yCITrace(IPOS, id(), "%d", joint);
    CHECK_JOINT(joint);
    *ref = trapTrajectory.getTargetPosition();
    return true;
}

// --------------------------------------------------------------------------------
