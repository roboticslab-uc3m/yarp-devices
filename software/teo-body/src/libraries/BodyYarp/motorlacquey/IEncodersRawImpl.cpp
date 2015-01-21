// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorLacquey.hpp"

// ------------------ IEncodersRaw Related -----------------------------------------

bool teo::MotorLacquey::resetEncoderRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return this->setEncoderRaw(j,0);
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::setEncoderRaw(int j, double val) {  // encExposed = val;
    CD_INFO("(%d,%f)\n",j,val);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (motorlacquey).\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::getEncoderRaw(int j, double *v) {
    //CD_INFO("%d\n",j);  //-- Too verbose in stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    encoderReady.wait();
    *v = encoder;
    encoderReady.post();

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::getEncoderSpeedRaw(int j, double *sp) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (motorlacquey).\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::getEncoderAccelerationRaw(int j, double *spds) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j = 0 ) return false;

    CD_WARNING("Not implemented yet (motorlacquey).\n");

    return true;
}

// -----------------------------------------------------------------------------

