// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// ############################ IEncodersRaw Related ############################

bool teo::TextilesHand::resetEncoderRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return this->setEncoderRaw(j,0);
}

// ------------------------------------------------------------------------------

bool teo::TextilesHand::resetEncodersRaw()
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool teo::TextilesHand::setEncoderRaw(int j, double val)    // encExposed = val;
{
    CD_INFO("(%d,%f)\n",j,val);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TextilesHand).\n");

    return true;
}

// ------------------------------------------------------------------------------

bool teo::TextilesHand::setEncodersRaw(const double *vals)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::getEncoderRaw(int j, double *v)
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

bool teo::TextilesHand::getEncodersRaw(double *encs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::getEncoderSpeedRaw(int j, double *sp)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    //CD_WARNING("Not implemented yet (TextilesHand).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *sp = 0;

    return true;
}

// ------------------------------------------------------------------------------

bool teo::TextilesHand::getEncoderSpeedsRaw(double *spds)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool teo::TextilesHand::getEncoderAccelerationRaw(int j, double *spds)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j = 0 ) return false;

    //CD_WARNING("Not implemented yet (TextilesHand).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *spds = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::getEncoderAccelerationsRaw(double *accs)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------
