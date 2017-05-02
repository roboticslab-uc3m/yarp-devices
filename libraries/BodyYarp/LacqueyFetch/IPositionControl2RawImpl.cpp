// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

// ######################## IPositionControlRaw Related ############################

bool teo::LacqueyFetch::getAxes(int *ax)
{
    *ax = 1;
    return true;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::positionMoveRaw(int j, double ref)    // encExposed = ref;
{
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;

    //Adjust range:
    if (ref > 1023)
    {
        ref=1023;
    }
    else if (ref < -1023)
    {
        ref=-1023;
    }

    //*************************************************************
    uint8_t msg_position_target[]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // Position target
    uint16_t pwm;
    //Send appropriate message:
    if (ref==0)
    {
        msg_position_target[0]=0xF0;
    }
    else if (ref>0)
    {
        pwm=ref;
        msg_position_target[1]=pwm>>2;
        msg_position_target[0]=0xA0 + (pwm & 0b0000000000000011);
    }
    else
    {
        pwm=abs(ref);
        msg_position_target[1]=pwm>>2;
        msg_position_target[0]=0xC0 + (pwm & 0b0000000000000011);
    }

    if( ! send( 0x600, 2, msg_position_target ) )
    {
        CD_ERROR("Could not send \"position target8\". %s\n", msgToStr(0x600, 2, msg_position_target).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position target8\". %s\n", msgToStr(0x600, 2, msg_position_target).c_str() );
    //*************************************************************

    encoderReady.wait();
    this->encoder = ref;  // Already passed through Adjust range.
    encoderReady.post();

    return true;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::positionMoveRaw(const double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::relativeMoveRaw(int j, double delta)
{
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (LacqueyFetch).\n");

    return true;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::relativeMoveRaw(const double *deltas)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::checkMotionDoneRaw(int j, bool *flag)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *flag = true;

    return true;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::checkMotionDoneRaw(bool *flag)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::setRefSpeedRaw(int j, double sp)
{
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::setRefSpeedsRaw(const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::setRefAccelerationRaw(int j, double acc)
{
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::setRefAccelerationsRaw(const double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::getRefSpeedRaw(int j, double *ref)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = 0;

    return true;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::getRefSpeedsRaw(double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::getRefAccelerationRaw(int j, double *acc)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = 0;

    return true;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::getRefAccelerationsRaw(double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::stopRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::stopRaw()
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ############################# IPositionControl2Raw Related ###########################

bool teo::LacqueyFetch::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::stopRaw(const int n_joint, const int *joints)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::getTargetPositionRaw(const int joint, double *ref)
{
    CD_INFO("\n");

    targetPositionSemaphore.wait();
    *ref = targetPosition;
    targetPositionSemaphore.post();

    return true;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::getTargetPositionsRaw(double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool teo::LacqueyFetch::getTargetPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}
