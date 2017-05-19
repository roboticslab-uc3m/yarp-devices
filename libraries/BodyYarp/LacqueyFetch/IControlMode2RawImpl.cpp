// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

// ############################## IControlModeRaw Related ##############################

bool teo::LacqueyFetch::setPositionModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::setVelocityModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::setTorqueModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::setImpedancePositionModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::setImpedanceVelocityModeRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::setOpenLoopModeRaw(int j)
{
    CD_ERROR("(%d)\n",j);  //-- Removed in YARP 2.3.70
    return false;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::getControlModeRaw(int j, int *mode)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream

    //-- Check index within range
    if ( j != 0 ) return false;

    *mode = VOCAB_CM_POSITION;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::getControlModesRaw(int *modes)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ############################## IControlMode2Raw Related ##############################

bool teo::LacqueyFetch::getControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::setControlModeRaw(const int j, const int mode)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::setControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::setControlModesRaw(int *modes)
{
    CD_ERROR("Missing implementation\n");
    return false;
}
