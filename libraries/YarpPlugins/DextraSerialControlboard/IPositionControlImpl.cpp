// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraSerialControlboard.hpp"

#include <cstring>

#include <algorithm>

#include <ColorDebug.h>

// ------------------- IPositionControl Related ------------------------------------

bool roboticslab::DextraSerialControlboard::getAxes(int *ax)
{
    *ax = Synapse::DATA_POINTS;
    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::positionMove(int j, double ref)    // encExposed = ref;
{
    CD_DEBUG("(%d, %f)\n", j, ref);
    CHECK_JOINT(j);

    Synapse::Setpoints setpoints;
    getSetpoints(setpoints);
    setpoints[j] = ref;

    if (!synapse->writeSetpointList(setpoints))
    {
        return false;
    }

    setSetpoint(j, ref);
    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::positionMove(const double *refs)
{
    CD_DEBUG("\n");

    Synapse::Setpoints setpoints;
    std::copy(refs, refs + Synapse::DATA_POINTS, setpoints);

    if (!synapse->writeSetpointList(setpoints))
    {
        return false;
    }

    setSetpoints(setpoints);
    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::relativeMove(int j, double delta)
{
    CD_DEBUG("(%d, %f)\n",j,delta);
    CHECK_JOINT(j);

    double ref;

    if (!getEncoder(j, &ref))
    {
        return false;
    }

    return positionMove(j, ref + delta);
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::relativeMove(const double *deltas)
{
    CD_DEBUG("\n");

    double encs[Synapse::DATA_POINTS];

    if (!getEncoders(encs))
    {
        return false;
    }

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        encs[j] += deltas[j];
    }

    return positionMove(encs);
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::checkMotionDone(int j, bool *flag)
{
    CD_DEBUG("(%d)\n",j);
    CHECK_JOINT(j);
    *flag = true;
    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::checkMotionDone(bool *flag)
{
    CD_DEBUG("\n");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        bool localFlag;
        ok &= checkMotionDone(j, &localFlag);
        *flag &= localFlag;
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::setRefSpeed(int j, double sp)
{
    CD_DEBUG("(%d, %f)\n", j ,sp);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::setRefSpeeds(const double *spds)
{
    CD_DEBUG("\n");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= setRefSpeed(j, spds[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::setRefAcceleration(int j, double acc)
{
    CD_DEBUG("(%d, %f)\n", j, acc);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::setRefAccelerations(const double *accs)
{
    CD_DEBUG("\n");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= setRefAcceleration(j, accs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getRefSpeed(int j, double *ref)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getRefSpeeds(double *spds)
{
    CD_DEBUG("\n");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getRefSpeed(j, &spds[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getRefAcceleration(int j, double *acc)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getRefAccelerations(double *accs)
{
    CD_DEBUG("\n");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getRefAcceleration(j, &accs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::stop(int j)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::stop()
{
    CD_DEBUG("\n");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= stop(j);
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::positionMove(const int n_joint, const int *joints, const double *refs)
{
    CD_DEBUG("(%d)\n", n_joint);

    double encs[Synapse::DATA_POINTS];

    if (!getEncoders(encs))
    {
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        encs[joints[i]] = refs[i];
    }

    return positionMove(encs);
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    CD_DEBUG("(%d)\n", n_joint);

    double encs[Synapse::DATA_POINTS];

    if (!getEncoders(encs))
    {
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        encs[joints[i]] += deltas[i];
    }

    return positionMove(encs);
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    CD_DEBUG("(%d)\n", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        bool localFlag;
        ok &= checkMotionDone(joints[i], &localFlag);
        *flags &= localFlag;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    CD_DEBUG("(%d)\n", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setRefSpeed(joints[i], spds[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    CD_DEBUG("(%d)\n", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setRefAcceleration(joints[i], accs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    CD_DEBUG("(%d)\n", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getRefSpeed(joints[i], &spds[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    CD_DEBUG("(%d)\n", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getRefAcceleration(joints[i], &accs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::stop(const int n_joint, const int *joints)
{
    CD_DEBUG("(%d)\n", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= stop(joints[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getTargetPosition(const int joint, double *ref)
{
    CD_DEBUG("(%d)\n", joint);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getTargetPositions(double *refs)
{
    CD_DEBUG("\n");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getTargetPosition(j, &refs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    CD_DEBUG("(%d)\n", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getTargetPosition(joints[i], &refs[i]);
    }

    return ok;
}
