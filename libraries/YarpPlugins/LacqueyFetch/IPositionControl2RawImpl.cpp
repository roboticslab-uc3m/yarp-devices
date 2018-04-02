// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

// ######################## IPositionControlRaw Related ############################

bool roboticslab::LacqueyFetch::getAxes(int *ax)
{
    *ax = 1;
    return true;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::positionMoveRaw(int j, double ref)    // encExposed = ref;
{
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;

    /*  Hand values sended to mbed:
        0 - loose hand
        1 - open hand
       -1 - close hand
    */
    if (ref > 1)
    {
        ref=1;
    }
    else if (ref < -1)
    {
        ref= -1;
    }

    uint8_t msg_position_target[]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // Position target (8 bytes)

    msg_position_target[0] = ref;

    if( ! send( 0x600, 1, msg_position_target ) )
    {
        CD_ERROR("Could not send \"position target8\". %s\n", msgToStr(0x600, 1, msg_position_target).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position target8\". %s\n", msgToStr(0x600, 1, msg_position_target).c_str() );
    //*************************************************************

    encoderReady.wait();
    this->encoder = ref;  // Already passed through Adjust range.
    encoderReady.post();

    return true;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::positionMoveRaw(const double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::relativeMoveRaw(int j, double delta)
{
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (LacqueyFetch).\n");

    return true;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::relativeMoveRaw(const double *deltas)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::checkMotionDoneRaw(int j, bool *flag)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *flag = true;

    return true;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::checkMotionDoneRaw(bool *flag)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setRefSpeedRaw(int j, double sp)
{
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setRefSpeedsRaw(const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setRefAccelerationRaw(int j, double acc)
{
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setRefAccelerationsRaw(const double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getRefSpeedRaw(int j, double *ref)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = 0;

    return true;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getRefSpeedsRaw(double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getRefAccelerationRaw(int j, double *acc)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = 0;

    return true;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getRefAccelerationsRaw(double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::stopRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::stopRaw()
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ############################# IPositionControl2Raw Related ###########################

bool roboticslab::LacqueyFetch::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::stopRaw(const int n_joint, const int *joints)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTargetPositionRaw(const int joint, double *ref)
{
    CD_INFO("\n");

    targetPositionSemaphore.wait();
    *ref = targetPosition;
    targetPositionSemaphore.post();

    return true;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTargetPositionsRaw(double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTargetPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}
