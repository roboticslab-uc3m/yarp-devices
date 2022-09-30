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

    double initialPosition = vars.internalUnitsToDegrees(vars.lastEncoderRead->queryPosition());
    trapTrajectory.configure(vars.syncPeriod, initialPosition, ref, vars.refSpeed, vars.refAcceleration);

    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::positionMoveRaw(const double * refs)
{
    return positionMoveRaw(0, refs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::positionMoveRaw(int n_joint, const int * joints, const double * refs)
{
    return positionMoveRaw(joints[0], refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::relativeMoveRaw(int j, double delta)
{
    yCITrace(IPOS, id(), "%d %f", j, delta);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    double initialPosition = vars.internalUnitsToDegrees(vars.lastEncoderRead->queryPosition());
    trapTrajectory.configure(vars.syncPeriod, initialPosition, initialPosition + delta, vars.refSpeed, vars.refAcceleration);

    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::relativeMoveRaw(const double * deltas)
{
    return relativeMoveRaw(0, deltas[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::relativeMoveRaw(int n_joint, const int * joints, const double * deltas)
{
    return relativeMoveRaw(joints[0], deltas[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::checkMotionDoneRaw(int j, bool * flag)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    *flag = !trapTrajectory.isActive();
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::checkMotionDoneRaw(bool * flag)
{
    return checkMotionDoneRaw(0, flag);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::checkMotionDoneRaw(int n_joint, const int * joints, bool * flag)
{
    return checkMotionDoneRaw(joints[0], flag);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setRefSpeedRaw(int j, double sp)
{
    yCITrace(IPOS, id(), "%d %f", j, sp);
    CHECK_JOINT(j);

    if (sp < 0.0)
    {
        yCIWarning(IPOS, id()) << "Illegal negative speed provided:" << sp;
        return false;
    }
    else if (sp > vars.maxVel)
    {
        yCIWarning(IPOS, id()) << "Reference speed exceeds maximum velocity, i.e." << vars.maxVel.load();
        return false;
    }

    vars.refSpeed = sp;
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::setRefSpeedsRaw(const double * spds)
{
    return setRefSpeedRaw(0, spds[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::setRefSpeedsRaw(int n_joint, const int * joints, const double * spds)
{
    return setRefSpeedRaw(joints[0], spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setRefAccelerationRaw(int j, double acc)
{
    yCITrace(IPOS, id(), "%d %f", j, acc);
    CHECK_JOINT(j);

    if (acc < 0.0)
    {
        yCIWarning(IPOS, id()) << "Illegal negative acceleration provided:" << acc;
        return false;
    }

    vars.refAcceleration = acc;
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::setRefAccelerationsRaw(const double * accs)
{
    return setRefAccelerationRaw(0, accs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs)
{
    return setRefAccelerationRaw(joints[0], accs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefSpeedRaw(int j, double * ref)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    *ref = vars.refSpeed;
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefSpeedsRaw(double * spds)
{
    return getRefSpeedRaw(0, &spds[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefSpeedsRaw(int n_joint, const int * joints, double * spds)
{
    return getRefSpeedRaw(joints[0], &spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefAccelerationRaw(int j, double * acc)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    *acc = vars.refAcceleration;
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefAccelerationsRaw(double * accs)
{
    return getRefAccelerationRaw(0, &accs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefAccelerationsRaw(int n_joint, const int * joints, double * accs)
{
    return getRefAccelerationRaw(joints[0], &accs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::stopRaw(int j)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    return false;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::stopRaw()
{
    return stopRaw(0);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::stopRaw(int n_joint, const int * joints)
{
    return stopRaw(joints[0]);
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

bool TechnosoftIposExternal::getTargetPositionsRaw(double * refs)
{
    return getTargetPositionRaw(0, &refs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposExternal::getTargetPositionsRaw(int n_joint, const int * joints, double * refs)
{
    return getTargetPositionRaw(joints[0], &refs[0]);
}

// --------------------------------------------------------------------------------
