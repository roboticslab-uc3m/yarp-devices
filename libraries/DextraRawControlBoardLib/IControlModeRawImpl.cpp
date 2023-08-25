// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlBoard.hpp"

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getControlModeRaw(int j, int * mode)
{
    yCITrace(DEXTRA, id(), "%d", j);
    CHECK_JOINT(j);
    *mode = VOCAB_CM_POSITION;
    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getControlModesRaw(int * modes)
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getControlModeRaw(j, &modes[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::getControlModesRaw(int n_joint, const int * joints, int * modes)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getControlModeRaw(joints[i], &modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::setControlModeRaw(int j, int mode)
{
    yCITrace(DEXTRA, id(), "%d %s", j, yarp::os::Vocab32::decode(mode).c_str());
    CHECK_JOINT(j);
    return false; // don't allow any control modes other than position direct, for now
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::setControlModesRaw(int * modes)
{
    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= setControlModeRaw(j, modes[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlBoard::setControlModesRaw(int n_joint, const int * joints, int * modes)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setControlModeRaw(joints[i], modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
