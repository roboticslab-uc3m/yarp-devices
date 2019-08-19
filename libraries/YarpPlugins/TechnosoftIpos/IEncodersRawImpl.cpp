// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <stdint.h>

#include <cstring>

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

bool roboticslab::TechnosoftIpos::setEncoderRaw(int j, double val)
{
    CD_INFO("(%d,%f)\n",j,val);

    //-- Check index within range
    if ( j != 0 ) return false;

    int32_t data = val * this->tr * (encoderPulses / 360.0);

    return sdoClient->download("Set actual position", data, 0X2081);
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
        int32_t data;

        if (!sdoClient->upload("Position actual value", &data, 0x6064))
        {
            return false;
        }

        lastEncoderRead.update(data / ((encoderPulses / 360.0) * this->tr));

        *v = lastEncoderRead.queryPosition();
    }
    else
    {
        iEncodersTimedRawExternal->getEncoderRaw(0, v);
        lastEncoderRead.update(*v);
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

    *sp = lastEncoderRead.querySpeed();

    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderSpeedsRaw(double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderAccelerationRaw(int j, double *acc)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = lastEncoderRead.queryAcceleration();

    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderAccelerationsRaw(double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}
