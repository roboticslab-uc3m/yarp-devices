// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraSerialControlboard.hpp"

#include <yarp/os/Vocab.h>

#include <ColorDebug.h>

// ------------------- IControlMode Related ------------------------------------

bool roboticslab::DextraSerialControlboard::getControlMode(int j, int *mode)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream
    CHECK_JOINT(j);
    *mode = VOCAB_CM_POSITION;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getControlModes(int *modes)
{
    //CD_DEBUG("\n");  //-- Too verbose in controlboardwrapper2 stream

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getControlMode(j, &modes[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::getControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("%d\n", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getControlMode(joints[i], &modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::setControlMode(const int j, const int mode)
{
    CD_DEBUG("(%d, %s)\n", j, yarp::os::Vocab::decode(mode).c_str());
    CHECK_JOINT(j);
    return false; // don't allow control modes other than position direct, for onw
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::setControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setControlMode(joints[i], modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::setControlModes(int *modes)
{
    CD_DEBUG("\n");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= setControlMode(j, modes[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
