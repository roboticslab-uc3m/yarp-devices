// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// ############################## IPositionControlRaw Related ##############################

bool roboticslab::TextilesHand::getAxes(int *ax)
{
    *ax = 1;
    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::positionMoveRaw(int j, double ref)    // encExposed = ref;
{
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;

    unsigned char cmdByte;
    if (ref == 0)
        cmdByte = 'a';
    else if (ref == 1)
        cmdByte = 'b';
    else
        return false;
    int res = serialport_writebyte(fd, cmdByte);
    if(res==-1) return false;

    encoderReady.wait();
    this->encoder = ref;  // Already passed through Adjust range.
    encoderReady.post();

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::positionMoveRaw(const double *refs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::relativeMoveRaw(int j, double delta)
{
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TextilesHand).\n");

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::relativeMoveRaw(const double *deltas)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::checkMotionDoneRaw(int j, bool *flag)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *flag = true;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::checkMotionDoneRaw(bool *flag)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setRefSpeedRaw(int j, double sp)
{
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setRefSpeedsRaw(const double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setRefAccelerationRaw(int j, double acc)
{
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setRefAccelerationsRaw(const double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getRefSpeedRaw(int j, double *ref)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = 0;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getRefSpeedsRaw(double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getRefAccelerationRaw(int j, double *acc)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = 0;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getRefAccelerationsRaw(double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::stopRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::stopRaw()
{
    CD_ERROR("\n");
    return false;
}


// ############################## IPositionControl2Raw Related ##############################


bool roboticslab::TextilesHand::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::stopRaw(const int n_joint, const int *joints)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTargetPositionRaw(const int joint, double *ref)
{
    CD_INFO("\n");

    *ref = targetPosition;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTargetPositionsRaw(double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTargetPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

