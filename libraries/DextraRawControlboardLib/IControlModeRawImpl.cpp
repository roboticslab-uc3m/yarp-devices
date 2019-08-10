// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Vocab.h>

#include <ColorDebug.h>
#include "../DextraRawControlboardLib/DextraRawControlboard.hpp"

// ------------------- IControlMode Related ------------------------------------

bool roboticslab::DextraRawControlboard::getControlModeRaw(int j, int *mode)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream
    CHECK_JOINT(j);
    *mode = VOCAB_CM_POSITION;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::getControlModesRaw(int *modes)
{
    //CD_DEBUG("\n");  //-- Too verbose in controlboardwrapper2 stream

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getControlModeRaw(j, &modes[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::getControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("%d\n", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getControlModeRaw(joints[i], &modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::setControlModeRaw(const int j, const int mode)
{
    CD_DEBUG("(%d, %s)\n", j, yarp::os::Vocab::decode(mode).c_str());
    CHECK_JOINT(j);
    return false; // don't allow control modes other than position direct, for onw
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::setControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setControlModeRaw(joints[i], modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::setControlModesRaw(int *modes)
{
    CD_DEBUG("\n");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= setControlModeRaw(j, modes[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
