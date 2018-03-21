// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraHand.hpp"

// ############################ IEncodersRaw Related ############################

bool roboticslab::DextraHand::resetEncoderRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return this->setEncoderRaw(j,0);
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraHand::resetEncodersRaw()
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraHand::setEncoderRaw(int j, double val)    // encExposed = val;
{
    CD_INFO("(%d,%f)\n",j,val);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (DextraHand).\n");

    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraHand::setEncodersRaw(const double *vals)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::getEncoderRaw(int j, double *v)
{
    //CD_INFO("%d\n",j);  //-- Too verbose in stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    encoderReady.wait();
    *v = encoder;
    encoderReady.post();

    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraHand::getEncodersRaw(double *encs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::getEncoderSpeedRaw(int j, double *sp)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    //CD_WARNING("Not implemented yet (DextraHand).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *sp = 0;

    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraHand::getEncoderSpeedsRaw(double *spds)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraHand::getEncoderAccelerationRaw(int j, double *spds)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j = 0 ) return false;

    //CD_WARNING("Not implemented yet (DextraHand).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *spds = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::getEncoderAccelerationsRaw(double *accs)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------
