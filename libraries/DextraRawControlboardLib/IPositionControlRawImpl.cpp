// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <cstring>

#include <algorithm>

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::getAxes(int *ax)
{
    *ax = Synapse::DATA_POINTS;
    return true;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::positionMoveRaw(int j, double ref)
{
    yCITrace(DEXTRA, id(), "%d %f", j, ref);
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

bool DextraRawControlboard::positionMoveRaw(const double * refs)
{
    yCITrace(DEXTRA, id(), "");

    Synapse::Setpoints setpoints;
    std::copy(refs, refs + Synapse::DATA_POINTS, std::begin(setpoints));

    if (!synapse->writeSetpointList(setpoints))
    {
        return false;
    }

    setSetpoints(setpoints);
    return true;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::positionMoveRaw(int n_joint, const int * joints, const double * refs)
{
    yCITrace(DEXTRA, id(), "%d", n_joint);

    double encs[Synapse::DATA_POINTS];

    if (!getEncodersRaw(encs))
    {
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        encs[joints[i]] = refs[i];
    }

    return positionMoveRaw(encs);
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::relativeMoveRaw(int j, double delta)
{
    yCITrace(DEXTRA, id(), "%d %f", j, delta);
    CHECK_JOINT(j);

    double ref;

    if (!getEncoderRaw(j, &ref))
    {
        return false;
    }

    return positionMoveRaw(j, ref + delta);
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::relativeMoveRaw(const double * deltas)
{
    yCITrace(DEXTRA, id(), "");

    double encs[Synapse::DATA_POINTS];

    if (!getEncodersRaw(encs))
    {
        return false;
    }

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        encs[j] += deltas[j];
    }

    return positionMoveRaw(encs);
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::relativeMoveRaw(int n_joint, const int * joints, const double * deltas)
{
    yCITrace(DEXTRA, id(), "%d", n_joint);

    double encs[Synapse::DATA_POINTS];

    if (!getEncodersRaw(encs))
    {
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        encs[joints[i]] += deltas[i];
    }

    return positionMoveRaw(encs);
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::checkMotionDoneRaw(int j, bool * flag)
{
    yCITrace(DEXTRA, id(), "%d", j);
    CHECK_JOINT(j);
    *flag = true;
    return true;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::checkMotionDoneRaw(bool * flag)
{
    yCITrace(DEXTRA, id(), "");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        bool localFlag;
        ok &= checkMotionDoneRaw(j, &localFlag);
        *flag &= localFlag;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::checkMotionDoneRaw(int n_joint, const int * joints, bool * flag)
{
    yCITrace(DEXTRA, id(), "%d", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        bool localFlag;
        ok &= checkMotionDoneRaw(joints[i], &localFlag);
        *flag &= localFlag;
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::setRefSpeedRaw(int j, double sp)
{
    yCITrace(DEXTRA, id(), "%d %f", j ,sp);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::setRefSpeedsRaw(const double * spds)
{
    yCITrace(DEXTRA, id(), "");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= setRefSpeedRaw(j, spds[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::setRefSpeedsRaw(int n_joint, const int * joints, const double * spds)
{
    yCITrace(DEXTRA, id(), "");

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setRefSpeedRaw(joints[i], spds[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::setRefAccelerationRaw(int j, double acc)
{
    yCITrace(DEXTRA, id(), "%d %f", j, acc);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::setRefAccelerationsRaw(const double * accs)
{
    yCITrace(DEXTRA, id(), "");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= setRefAccelerationRaw(j, accs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs)
{
    yCITrace(DEXTRA, id(), "%d", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setRefAccelerationRaw(joints[i], accs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::getRefSpeedRaw(int j, double * ref)
{
    yCITrace(DEXTRA, id(), "%d", j);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::getRefSpeedsRaw(double * spds)
{
    yCITrace(DEXTRA, id(), "");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getRefSpeedRaw(j, &spds[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getRefSpeedsRaw(int n_joint, const int * joints, double * spds)
{
    yCITrace(DEXTRA, id(), "%d", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getRefSpeedRaw(joints[i], &spds[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::getRefAccelerationRaw(int j, double * acc)
{
    yCITrace(DEXTRA, id(), "%d", j);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::getRefAccelerationsRaw(double * accs)
{
    yCITrace(DEXTRA, id(), "");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getRefAccelerationRaw(j, &accs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getRefAccelerationsRaw(int n_joint, const int * joints, double * accs)
{
    yCITrace(DEXTRA, id(), "%d", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getRefAccelerationRaw(joints[i], &accs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::stopRaw(int j)
{
    yCITrace(DEXTRA, id(), "%d", j);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------------------

bool DextraRawControlboard::stopRaw()
{
    yCITrace(DEXTRA, id(), "");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= stopRaw(j);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::stopRaw(int n_joint, const int * joints)
{
    yCITrace(DEXTRA, id(), "%d", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= stopRaw(joints[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getTargetPositionRaw(int joint, double * ref)
{
    yCITrace(DEXTRA, id(), "%d", joint);
    return false;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getTargetPositionsRaw(double * refs)
{
    yCITrace(DEXTRA, id(), "");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getTargetPositionRaw(j, &refs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getTargetPositionsRaw(int n_joint, const int * joints, double * refs)
{
    yCITrace(DEXTRA, id(), "%d", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getTargetPositionRaw(joints[i], &refs[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
