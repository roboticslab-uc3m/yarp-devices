// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// -------------------------- IEncodersRaw Related ----------------------------------

bool roboticslab::TechnosoftIpos::resetEncoderRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return this->setEncoderRaw(j,0);
}

bool roboticslab::TechnosoftIpos::resetEncodersRaw()
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setEncoderRaw(int j, double val)    // encExposed = val;
{
    CD_INFO("(%d,%f)\n",j,val);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_setEncoder[]= {0x23,0x81,0x20,0x00,0x00,0x00,0x00,0x00}; // Manual 2081h: Set/Change the actual motor position

    int sendEnc = val * this->tr * (encoderPulses / 360.0);  // Apply tr & convert units to encoder increments
    memcpy(msg_setEncoder+4,&sendEnc,4);

    if( ! send(0x600, 8, msg_setEncoder))
    {
        CD_ERROR("Sent \"set encoder\". %s\n", msgToStr(0x600, 8, msg_setEncoder).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"set encoder\". %s\n", msgToStr(0x600, 8, msg_setEncoder).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setEncodersRaw(const double *vals)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderRaw(int j, double *v)
{
    //CD_INFO("%d\n",j);  //-- Too verbose in stream.

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
        *v = encoder;
        encoderReady.post();

        //*************************************************************
    }
    else
    {
        iEncodersTimedRawExternal->getEncoderRaw(0,v);
    }

    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncodersRaw(double *encs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderSpeedRaw(int j, double *sp)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    //CD_WARNING("Not implemented yet (TechnosoftIpos).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *sp = 0;

    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderSpeedsRaw(double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderAccelerationRaw(int j, double *spds)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j = 0 ) return false;

    //CD_WARNING("Not implemented yet (TechnosoftIpos).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *spds = 0;

    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderAccelerationsRaw(double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}
