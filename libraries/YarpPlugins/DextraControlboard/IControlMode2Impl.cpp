// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboard.hpp"

// ############################## IControlMode Related ##############################

bool roboticslab::DextraControlboard::getControlMode(int j, int *mode)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream

    //-- Check index within range
    if ( j != 0 ) return false;

    *mode = VOCAB_CM_POSITION;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getControlModes(int *modes)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ############################## IControlMode2 Related ##############################

bool roboticslab::DextraControlboard::getControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("\n");

    //-- Check array size
    if ( n_joint != 1 ) return false;

    return getControlMode(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setControlMode(const int j, const int mode)
{
    CD_DEBUG("(%d, %d)\n",j,mode);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n",n_joint);

    //-- Check array size
    if ( n_joint != 1 ) return false;

    return setControlMode(0, modes[0]);
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setControlModes(int *modes)
{
    CD_DEBUG("\n");
    return setControlMode(0, modes[0]);
}
