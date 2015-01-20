// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorLacquey.hpp"

// ------------------ IEncodersTimedRaw Related -----------------------------------------

bool teo::MotorLacquey::getEncoderTimedRaw(int j, double *encs, double *time) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_read[]={0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00}; // Query position.
    if( ! send( 0x600, 8, msg_read) )
    {
        CD_ERROR("Could not send \"read encoder\". %s\n", msgToStr(0x600, 8, msg_read).c_str() );
        return false;
    }
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    Time::delay(DELAY);  // Must delay as it will be from same driver.
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    encoderReady.wait();
    *encs = encoder;
    *time = encoderTimestamp;
    encoderReady.post();

    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------
