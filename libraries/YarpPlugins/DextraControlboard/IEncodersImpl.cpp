// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboard.hpp"

// ############################ IEncoders Related ############################

bool roboticslab::DextraControlboard::resetEncoder(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return this->setEncoder(j,0);
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::resetEncoders()
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setEncoder(int j, double val)    // encExposed = val;
{
    CD_INFO("(%d,%f)\n",j,val);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (DextraControlboard).\n");

    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setEncoders(const double *vals)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getEncoder(int j, double *v)
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

bool roboticslab::DextraControlboard::getEncoders(double *encs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getEncoderSpeed(int j, double *sp)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    //CD_WARNING("Not implemented yet (DextraControlboard).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *sp = 0;

    return true;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getEncoderSpeeds(double *spds)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getEncoderAcceleration(int j, double *spds)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j = 0 ) return false;

    //CD_WARNING("Not implemented yet (DextraControlboard).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *spds = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getEncoderAccelerations(double *accs)
{
    CD_ERROR("\n");
    return false;
}

// ------------------------------------------------------------------------------
