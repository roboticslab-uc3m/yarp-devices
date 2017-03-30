// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IEncodersTimedRaw Related -----------------------------------------

bool teo::TechnosoftIpos::getEncodersTimedRaw(double *encs, double *time)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//---------------------------------------------------------------------------------------

bool teo::TechnosoftIpos::getEncoderTimedRaw(int j, double *encs, double *time)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    if( ! iEncodersTimedRawExternal )
    {
        //*************************************************************
        uint8_t msg_read[]= {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00}; // Query position.
        if( ! send( 0x600, 8, msg_read) )
        {
            CD_ERROR("Could not send \"read encoder\". %s\n", msgToStr(0x600, 8, msg_read).c_str() );
            return false;
        }
        //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
        yarp::os::Time::delay(DELAY);  // Must delay as it will be from same driver.
        //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

        encoderReady.wait();
        *encs = encoder;
        *time = encoderTimestamp;
        encoderReady.post();

        //*************************************************************
    }
    else
    {
        iEncodersTimedRawExternal->getEncoderTimedRaw(0,encs,time);
    }

    return true;
}

// -----------------------------------------------------------------------------
