// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <yarp/os/Vocab.h>

// ############################## IControlModeRaw Related ##############################

bool roboticslab::LacqueyFetch::getControlModeRaw(int j, int *mode)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream

    //-- Check index within range
    if ( j != 0 ) return false;

    *mode = VOCAB_CM_POSITION;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getControlModesRaw(int *modes)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ############################## IControlMode2Raw Related ##############################

bool roboticslab::LacqueyFetch::getControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("\n");

    //-- Check array size
    if ( n_joint != 1 ) return false;

    return getControlModeRaw(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setControlModeRaw(const int j, const int mode)
{
    CD_DEBUG("(%d, %s)\n", j, yarp::os::Vocab::decode(mode).c_str());

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n",n_joint);

    //-- Check array size
    if ( n_joint != 1 ) return false;

    return setControlModeRaw(0, modes[0]);
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setControlModesRaw(int *modes)
{
    CD_DEBUG("\n");
    return setControlModeRaw(0, modes[0]);
}
