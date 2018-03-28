// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboard.hpp"

// ############################## IPositionControl Related ##############################

bool roboticslab::DextraControlboard::getAxes(int *ax)
{
    *ax = 6;
    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::positionMove(int j, double ref)    // encExposed = ref;
{
    CD_INFO("(%d,%f)\n",j,ref);

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

bool roboticslab::DextraControlboard::positionMove(const double *refs)
{
    CD_INFO("\n");

    uint8_t cmdByte;

    //-- Send header
    cmdByte = 0x7E;
    if( serialport_writebyte(fd, cmdByte) != 0 )
    {
        CD_ERROR("Failed to send header\n");
        return false;
    }

    for(int j=0; j<6;j++)
    {
        //-- Send address
        //-- Addresses: chr(0x01),chr(0x02),chr(0x03),chr(0x04),chr(0x05),chr(0x06)
        cmdByte = static_cast<uint8_t>(j+1);  // tad bit hacki-ish
        if( serialport_writebyte(fd, cmdByte) != 0 )
        {
            CD_ERROR("Failed to send address\n");
            return false;
        }

        //-- Send targets
        size_t size = sizeof(float);
        float refFloat = static_cast<float>(refs[j]);

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
    }

    //-- Send footer
    cmdByte = 0x7E;
    if( serialport_writebyte(fd, cmdByte) != 0 )
    {
        CD_ERROR("Failed to send footer\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::relativeMove(int j, double delta)
{
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (DextraControlboard).\n");

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::relativeMove(const double *deltas)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::checkMotionDone(int j, bool *flag)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *flag = true;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::checkMotionDone(bool *flag)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefSpeed(int j, double sp)
{
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefSpeeds(const double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefAcceleration(int j, double acc)
{
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefAccelerations(const double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefSpeed(int j, double *ref)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = 0;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefSpeeds(double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefAcceleration(int j, double *acc)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = 0;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefAccelerations(double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::stop(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::stop()
{
    CD_ERROR("\n");
    return false;
}


// ############################## IPositionControl2 Related ##############################


bool roboticslab::DextraControlboard::positionMove(const int n_joint, const int *joints, const double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::stop(const int n_joint, const int *joints)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTargetPosition(const int joint, double *ref)
{
    CD_INFO("\n");

    *ref = targetPosition;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTargetPositions(double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

