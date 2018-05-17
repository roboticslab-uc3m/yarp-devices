// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

// ############################## IControlModeRaw Related ##############################

bool roboticslab::CuiAbsolute::setPositionModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setVelocityModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setTorqueModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setImpedancePositionModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setImpedanceVelocityModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getControlModeRaw(int j, int *mode)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream

    //-- Check index within range
    if ( j != 0 ) return false;

    *mode = VOCAB_CM_POSITION;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getControlModesRaw(int *modes)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ############################## IControlMode2Raw Related ##############################

bool roboticslab::CuiAbsolute::getControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("\n");

    //-- Check array size
    if ( n_joint != 1 ) return false;

    return getControlModeRaw(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setControlModeRaw(const int j, const int mode)
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

    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n",n_joint);

    //-- Check array size
    if ( n_joint != 1 ) return false;

    return setControlModeRaw(0, modes[0]);
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setControlModesRaw(int *modes)
{
    CD_DEBUG("\n");
    return setControlModeRaw(0, modes[0]);
}
