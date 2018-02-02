// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

// ######################### IEncodersRaw Related ##############################

bool roboticslab::CuiAbsolute::resetEncoderRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return this->setEncoderRaw(j,0);
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::resetEncodersRaw()
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setEncoderRaw(int j, double val)    // encExposed = val;
{
    CD_INFO("(%d,%f)\n",j,val);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (CuiAbsolute).\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setEncodersRaw(const double *vals)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getEncoderRaw(int j, double *v)
{
    //CD_INFO("%d\n",j);  //-- Too verbose in stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    encoderReady.wait();
    *v = encoder;
    encoderReady.post();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getEncodersRaw(double *encs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getEncoderSpeedRaw(int j, double *sp)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    //CD_WARNING("Not implemented yet (CuiAbsolute).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *sp = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getEncoderSpeedsRaw(double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getEncoderAccelerationRaw(int j, double *spds)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j = 0 ) return false;

    //CD_WARNING("Not implemented yet (CuiAbsolute).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *spds = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getEncoderAccelerationsRaw(double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

