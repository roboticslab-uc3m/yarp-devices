// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboard.hpp"

// ############################## IPositionControlRaw Related ##############################

bool roboticslab::DextraControlboard::getAxes(int *ax)
{
    *ax = 1;
    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::positionMoveRaw(int j, double ref)    // encExposed = ref;
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
    size_t size = sizeof(float);
    float refFloat = static_cast<float>(ref);

    std::vector<uint8_t> msg_position_target;
    msg_position_target.resize(size,0);

    memcpy(msg_position_target.data(),&refFloat,size);

    for( size_t i = 0; i<size; i++)
    {
        cmdByte = msg_position_target[i];
        if( serialport_writebyte(fd, cmdByte) != 0 )
        {
            CD_ERROR("Failed to send target [%d of %d]\n",i,size);
            return false;
        }
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

bool roboticslab::DextraControlboard::positionMoveRaw(const double *refs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::relativeMoveRaw(int j, double delta)
{
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (DextraControlboard).\n");

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::relativeMoveRaw(const double *deltas)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::checkMotionDoneRaw(int j, bool *flag)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *flag = true;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::checkMotionDoneRaw(bool *flag)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefSpeedRaw(int j, double sp)
{
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefSpeedsRaw(const double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefAccelerationRaw(int j, double acc)
{
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefAccelerationsRaw(const double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefSpeedRaw(int j, double *ref)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = 0;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefSpeedsRaw(double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefAccelerationRaw(int j, double *acc)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = 0;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefAccelerationsRaw(double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::stopRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::stopRaw()
{
    CD_ERROR("\n");
    return false;
}


// ############################## IPositionControl2Raw Related ##############################


bool roboticslab::DextraControlboard::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::stopRaw(const int n_joint, const int *joints)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTargetPositionRaw(const int joint, double *ref)
{
    CD_INFO("\n");

    *ref = targetPosition;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTargetPositionsRaw(double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTargetPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

