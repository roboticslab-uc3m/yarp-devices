// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboard.hpp"

// ############################## IControlModeRaw Related ##############################

bool roboticslab::DextraControlboard::setPositionModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setVelocityModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setTorqueModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setImpedancePositionModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setImpedanceVelocityModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setOpenLoopModeRaw(int j)
{
    CD_ERROR("(%d)\n",j);  //-- Removed in YARP 2.3.70
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getControlModeRaw(int j, int *mode)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream

    //-- Check index within range
    if ( j != 0 ) return false;

    *mode = VOCAB_CM_POSITION;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getControlModesRaw(int *modes)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ############################## IControlMode2Raw Related ##############################

bool roboticslab::DextraControlboard::getControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("\n");

    //-- Check array size
    if ( n_joint != 1 ) return false;

    return getControlModeRaw(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setControlModeRaw(const int j, const int mode)
{
    CD_DEBUG("(%d, %d)\n",j,mode);

    //-- Check index within range
    if ( j != 0 ) return false;

    if( mode == VOCAB_CM_POSITION )
        return setPositionModeRaw(j);
    else if( mode == VOCAB_CM_VELOCITY )
        return setVelocityModeRaw(j);
    else if( mode == VOCAB_CM_TORQUE )
        return setTorqueModeRaw(j);
    else if( mode == VOCAB_CM_IMPEDANCE_POS )
        return setImpedancePositionModeRaw(j);
    else if( mode == VOCAB_CM_IMPEDANCE_VEL )
        return setImpedanceVelocityModeRaw(j);
    /*else if( mode == VOCAB_CM_OPENLOOP )
        return setOpenLoopModeRaw(j);*/

    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n",n_joint);

    //-- Check array size
    if ( n_joint != 1 ) return false;

    return setControlModeRaw(0, modes[0]);
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setControlModesRaw(int *modes)
{
    CD_DEBUG("\n");
    return setControlModeRaw(0, modes[0]);
}
