// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorLacquey.hpp"

// ------------------- IControlModeRaw Related ------------------------------------

bool teo::MotorLacquey::setPositionModeRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::setVelocityModeRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::setTorqueModeRaw(int j)  {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::setImpedancePositionModeRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::setImpedanceVelocityModeRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::setOpenLoopModeRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::getControlModeRaw(int j, int *mode) {
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream

    //-- Check index within range
    if ( j != 0 ) return false;

    *mode = VOCAB_POSITION_MODE;

    return true;
}

// -----------------------------------------------------------------------------
