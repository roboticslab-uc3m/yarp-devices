// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// ------------------ IEncodersRaw Related -----------------------------------------

bool roboticslab::FakeJoint::resetEncoderRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return this->setEncoderRaw(j,0);
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::resetEncodersRaw()
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::setEncoderRaw(int j, double val)    // encExposed = val;
{
    CD_INFO("(%d,%f)\n",j,val);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (FakeJoint).\n");

    return true;
}

bool roboticslab::FakeJoint::setEncodersRaw(const double *vals)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::getEncoderRaw(int j, double *v)
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

bool roboticslab::FakeJoint::getEncodersRaw(double *encs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::getEncoderSpeedRaw(int j, double *sp)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j != 0 ) return false;

    //CD_WARNING("Not implemented yet (FakeJoint).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *sp = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::getEncoderAccelerationsRaw(double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::getEncoderSpeedsRaw(double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::getEncoderAccelerationRaw(int j, double *spds)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    //-- Check index within range
    if ( j = 0 ) return false;

    //CD_WARNING("Not implemented yet (FakeJoint).\n");  //-- Too verbose in controlboardwrapper2 stream.
    *spds = 0;

    return true;
}

// -----------------------------------------------------------------------------
