// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// ############################## IPositionControlRaw Related ##############################

bool teo::TextilesHand::getAxes(int *ax)
{
    *ax = 1;
    return true;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::setPositionModeRaw()
{
    return setPositionModeRaw(0);
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::positionMoveRaw(int j, double ref)    // encExposed = ref;
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

bool teo::TextilesHand::positionMoveRaw(const double *refs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::relativeMoveRaw(int j, double delta)
{
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TextilesHand).\n");

    return true;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::relativeMoveRaw(const double *deltas)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::checkMotionDoneRaw(int j, bool *flag)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *flag = true;

    return true;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::checkMotionDoneRaw(bool *flag)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::setRefSpeedRaw(int j, double sp)
{
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::setRefSpeedsRaw(const double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::setRefAccelerationRaw(int j, double acc)
{
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::setRefAccelerationsRaw(const double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::getRefSpeedRaw(int j, double *ref)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = 0;

    return true;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::getRefSpeedsRaw(double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::getRefAccelerationRaw(int j, double *acc)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = 0;

    return true;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::getRefAccelerationsRaw(double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::stopRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool teo::TextilesHand::stopRaw()
{
    CD_ERROR("\n");
    return false;
}


// ############################## IPositionControl2Raw Related ##############################


bool teo::TextilesHand::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::stopRaw(const int n_joint, const int *joints)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::getTargetPositionRaw(const int joint, double *ref)
{
    CD_INFO("\n");

    *ref = targetPosition;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::getTargetPositionsRaw(double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::getTargetPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

