// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorLacquey.hpp"

// ------------------ IEncodersTimedRaw Related -----------------------------------------

bool teo::MotorLacquey::getEncoderTimedRaw(int j, double *encs, double *time) {
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------
