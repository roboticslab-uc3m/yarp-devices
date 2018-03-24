// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraHand.hpp"

// ############################## IPositionControlRaw Related ##############################

bool roboticslab::DextraHand::getAxes(int *ax)
{
    *ax = 1;
    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::positionMoveRaw(int j, double ref)    // encExposed = ref;
{
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;

    uint8_t cmdByte;

    //-- Send header
    cmdByte = 0x7E;
    if( serialport_writebyte(fd, cmdByte) != 0 )
    {
        CD_ERROR("Failed to send header\n");
        return false;
    }

    //-- Send address
    //-- Addresses: chr(0x01),chr(0x02),chr(0x03),chr(0x04),chr(0x05),chr(0x06)
    cmdByte = static_cast<uint8_t>(j+1);  // tad bit hacki-ish
    if( serialport_writebyte(fd, cmdByte) != 0 )
    {
        CD_ERROR("Failed to send address\n");
        return false;
    }

    //-- Send target
    cmdByte = ref;
    if( serialport_writebyte(fd, cmdByte) != 0 )
    {
        CD_ERROR("Failed to send target\n");
        return false;
    }

    //-- Send footer
    cmdByte = 0x7E;
    if( serialport_writebyte(fd, cmdByte) != 0 )
    {
        CD_ERROR("Failed to send footer\n");
        return false;
    }

    encoderReady.wait();
    this->encoder = ref;  // Already passed through Adjust range.
    encoderReady.post();

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::positionMoveRaw(const double *refs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::relativeMoveRaw(int j, double delta)
{
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (DextraHand).\n");

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::relativeMoveRaw(const double *deltas)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::checkMotionDoneRaw(int j, bool *flag)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *flag = true;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::checkMotionDoneRaw(bool *flag)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::setRefSpeedRaw(int j, double sp)
{
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::setRefSpeedsRaw(const double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::setRefAccelerationRaw(int j, double acc)
{
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::setRefAccelerationsRaw(const double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::getRefSpeedRaw(int j, double *ref)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = 0;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::getRefSpeedsRaw(double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::getRefAccelerationRaw(int j, double *acc)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = 0;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::getRefAccelerationsRaw(double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::stopRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraHand::stopRaw()
{
    CD_ERROR("\n");
    return false;
}


// ############################## IPositionControl2Raw Related ##############################


bool roboticslab::DextraHand::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::stopRaw(const int n_joint, const int *joints)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::getTargetPositionRaw(const int joint, double *ref)
{
    CD_INFO("\n");

    *ref = targetPosition;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::getTargetPositionsRaw(double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::getTargetPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

