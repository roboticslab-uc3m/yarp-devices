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

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::setImpedanceVelocityModeRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::setOpenLoopModeRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::getControlModeRaw(int j, int *mode) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------
