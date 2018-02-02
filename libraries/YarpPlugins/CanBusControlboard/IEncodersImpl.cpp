// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------ IEncoders Related -----------------------------------------

bool roboticslab::CanBusControlboard::resetEncoder(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iEncodersTimedRaw[j]->resetEncoderRaw( 0 );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::resetEncoders()
{
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i < nodes.size(); i++)
        ok &= resetEncoder(i);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setEncoder(int j, double val)
{
    CD_INFO("(%d,%f)\n",j,val);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iEncodersTimedRaw[j]->setEncoderRaw( 0, val );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setEncoders(const double *vals)
{
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i < nodes.size(); i++)
        ok &= setEncoder(i,vals[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getEncoder(int j, double *v)
{
    //CD_INFO("%d\n",j);  //-- Too verbose in stream.

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iEncodersTimedRaw[j]->getEncoderRaw( 0, v );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getEncoders(double *encs)
{
    //CD_INFO("\n");  //-- Too verbose in stream.

    bool ok = true;
    for(unsigned int i=0; i < nodes.size(); i++)
        ok &= getEncoder(i,&(encs[i]));
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getEncoderSpeed(int j, double *sp)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iEncodersTimedRaw[j]->getEncoderSpeedRaw( 0, sp );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getEncoderSpeeds(double *spds)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
        ok &= getEncoderSpeed(i,&spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getEncoderAcceleration(int j, double *spds)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iEncodersTimedRaw[j]->getEncoderAccelerationRaw( 0, spds );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getEncoderAccelerations(double *accs)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
        ok &= getEncoderAcceleration(i,&accs[i]);
    return ok;
}

// -----------------------------------------------------------------------------

