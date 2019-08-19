// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>

#include <bitset>

#include "CanUtils.hpp"

// ######################### IPositionControlRaw Related #########################

bool roboticslab::TechnosoftIpos::getAxes(int *ax)
{
    *ax = 1;
    return true;
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::positionMoveRaw(int j, double ref)    // encExposed = ref;
{
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;

    //-- Sets "Do not assume target position" so later it accepts "Assume target position (update the new motion parameters)".
    //-- Mandatory if we issue this command right after the transition from [posd] to [pos] control mode.
    //*************************************************************
    uint8_t msg_pos_reset[] = {0x0F,0x00}; // Stop a position profile

    if( ! send( 0x200, 2, msg_pos_reset) )
    {
        CD_ERROR("Could not send first \"reset position\". %s\n", msgToStr(0x200, 2, msg_pos_reset).c_str() );
        return false;
    }
    CD_SUCCESS("Sent first \"reset position\". %s\n", msgToStr(0x200, 2, msg_pos_reset).c_str() );

    //*************************************************************

    int32_t data = ref * tr * (encoderPulses / 360.0);

    if (!sdoClient->download("Target position", data, 0x607A))
    {
        return false;
    }

    //*************************************************************
    //uint8_t msg_start[]={0x1F,0x00}; // Start the movement with "Discrete motion profile (change set immediately = 0)".
    uint8_t msg_start[]= {0x3F,0x00}; // Start the movement with "Continuous motion profile (change set immediately = 1)".

    if( ! send( 0x200, 2, msg_start ) )
    {
        CD_ERROR("Could not send \"start position\". %s\n", msgToStr(0x200, 2, msg_start).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"start position\". %s\n", msgToStr(0x200, 2, msg_start).c_str() );

    //*************************************************************

    //-- Needed to accept next target. Sets "Do not assume target position" so later it accepts "Assume target position (update the new motion parameters)".
    if( ! send( 0x200, 2, msg_pos_reset) )
    {
        CD_ERROR("Could not send second \"reset position\". %s\n", msgToStr(0x200, 2, msg_pos_reset).c_str() );
        return false;
    }
    CD_SUCCESS("Sent second \"reset position\". %s\n", msgToStr(0x200, 2, msg_pos_reset).c_str() );

    return true;
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::positionMoveRaw(const double *refs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::relativeMoveRaw(int j, double delta)
{
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    int32_t data = delta * tr * (encoderPulses / 360.0);

    if (!sdoClient->download("Target position", data, 0x607A))
    {
        return false;
    }

    //*************************************************************
    //uint8_t msg_start_rel[]={0x5F,0x00}; // Start the movement with "Discrete motion profile (change set immediately = 0)".
    uint8_t msg_start_rel[]= {0x7F,0x00}; // Start the movement with "Continuous motion profile (change set immediately = 1)".


    if( ! send( 0x200, 2, msg_start_rel ) )
    {
        CD_ERROR("Could not send \"start rel position. %s\n", msgToStr(0x200, 2, msg_start_rel).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"start rel position\". %s\n", msgToStr(0x200, 2, msg_start_rel).c_str() );;
    //*************************************************************

    //-- Needed to send next. Sets "Do not assume target position" so later it accepts "Assume target position (update the new motion parameters)".
    //*************************************************************
    uint8_t msg_pos_reset[]= {0x0F,0x00}; // Stop a position profile

    if( ! send( 0x200, 2, msg_pos_reset) )
    {
        CD_ERROR("Could not send \"reset position\". %s\n", msgToStr(0x200, 2, msg_pos_reset).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"reset position\". %s\n", msgToStr(0x200, 2, msg_pos_reset).c_str() );
    //*************************************************************

    return true;
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::relativeMoveRaw(const double *deltas)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::checkMotionDoneRaw(int j, bool *flag)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    uint16_t data;

    if (!sdoClient->upload("Statusword", &data, 0x6041))
    {
        return false;
    }

    std::bitset<16> bits(data);

    if (bits.test(0))
    {
        CD_INFO("\t-Ready to switch on. canId: %d.\n",canId);
    }
    if (bits.test(1))
    {
        CD_INFO("\t-Switched on. canId: %d.\n",canId);
    }
    if (bits.test(2))
    {
        CD_INFO("\t-Operation Enabled. canId: %d.\n",canId);
    }
    if (bits.test(3))
    {
        CD_INFO("\t-Fault. If set, a fault condition is or was present in the drive. canId: %d.\n",canId);
    }
    if (bits.test(4))
    {
        CD_INFO("\t-Motor supply voltage is present. canId: %d.\n",canId);
    }
    else
    {
        CD_INFO("\t-Motor supply voltage is absent. canId: %d.\n",canId);
    }
    if (!bits.test(5)) // negated.
    {
        CD_INFO("\t-Performing a quick stop. canId: %d.\n",canId);
    }
    if (bits.test(6))
    {
        CD_INFO("\t-Switch on disabled. canId: %d.\n",canId);
    }
    if (bits.test(7))
    {
        CD_INFO("\t-Warning. A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored. canId: %d.\n",canId);
    }
    if (bits.test(8))
    {
        CD_INFO("\t-A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called. canId: %d.\n",canId);
    }
    if (bits.test(9))
    {
        CD_INFO("\t-Remote: drive parameters may be modified via CAN and the drive will execute the command message. canId: %d.\n",canId);
    }
    else
    {
        CD_INFO("\t-Remote: drive is in local mode and will not execute the command message (only TML internal).");
    }
    if (bits.test(10))
    {
        CD_INFO("\t-Target reached. canId: %d.\n",canId);
    }
    else
    {
        CD_INFO("\t-Target not reached. canId: %d.\n",canId);  // improvised, not in manual, but reasonable
    }
    if (bits.test(11))
    {
        CD_INFO("\t-Internal Limit Active. canId: %d.\n",canId);
    }
    if (bits.test(14))
    {
        CD_INFO("\t-Last event set has ocurred. canId: %d.\n",canId);
    }
    else
    {
        CD_INFO("\t-No event set or the programmed event has not occurred yet. canId: %d.\n",canId);
    }
    if (bits.test(15))
    {
        CD_INFO("\t-Axis on. Power stage is enabled. Motor control is performed. canId: %d.\n",canId);
    }
    else
    {
        CD_INFO("\t-Axis off. Power stage is disabled. Motor control is not performed. canId: %d.\n",canId);
    }

    *flag = bits.test(10);
    return true;
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::checkMotionDoneRaw(bool *flag)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefSpeedRaw(int j, double sp)
{
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    if ( sp > maxVel )
    {
        CD_WARNING("Reference speed exceeds maximum velocity (%f).\n", maxVel);
        return false;
    }

    double value = sp * std::abs(tr) * (encoderPulses / 360.0) * 0.001;

    uint16_t dataInt;
    uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    uint32_t data = (dataInt << 16) + dataFrac;
    return sdoClient->download("Profile velocity", data, 0x6081);
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefSpeedsRaw(const double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefAccelerationRaw(int j, double acc)
{
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    double value = acc * std::abs(tr) * (encoderPulses / 360.0) * 0.000001;

    uint16_t dataInt;
    uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    uint32_t data = (dataInt << 16) + dataFrac;
    return sdoClient->download("Profile acceleration", data, 0x6083);
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefAccelerationsRaw(const double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefSpeedRaw(int j, double *ref)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    uint32_t data;

    if (!sdoClient->upload("Profile velocity", &data, 0x6081))
    {
        return false;
    }

    uint16_t dataInt = data >> 16;
    uint16_t dataFrac = data & 0xFFFF;
    double value = CanUtils::decodeFixedPoint(dataInt, dataFrac);

    *ref = value / (std::abs(tr) * (encoderPulses / 360.0) * 0.001);
    return true;
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefSpeedsRaw(double *spds)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefAccelerationRaw(int j, double *acc)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    uint32_t data;

    if (!sdoClient->upload("Profile acceleration", &data, 0x6083))
    {
        return false;
    }

    uint16_t dataInt = data >> 16;
    uint16_t dataFrac = data & 0xFFFF;
    double value = CanUtils::decodeFixedPoint(dataInt, dataFrac);

    *acc = value / (std::abs(tr) * (encoderPulses / 360.0) * 0.000001);
    return true;
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefAccelerationsRaw(double *accs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::stopRaw(int j)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    uint8_t msg_quickStop[] = {0x02,0x00};

    if (!this->send(0x200, 2, msg_quickStop))
    {
        CD_ERROR("Could not send \"quick stop\". %s\n", msgToStr(0x200, 2, msg_quickStop).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"quick stop\". %s\n", msgToStr(0x200, 2, msg_quickStop).c_str() );

    yarp::os::Time::delay(0.01);

    return enable();
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::stopRaw()
{
    CD_ERROR("\n");
    return false;
}

// ######################### IPositionControl2Raw Related #########################

bool roboticslab::TechnosoftIpos::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// --------------------------------------------------------------------------------
/*
bool roboticslab::TechnosoftIpos::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    return true;
}
*/

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// --------------------------------------------------------------------------------

/*
bool roboticslab::TechnosoftIpos::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    return true;
}
*/

// --------------------------------------------------------------------------------

/*
bool roboticslab::TechnosoftIpos::stopRaw(const int n_joint, const int *joints)
{
    return true;
}
*/

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTargetPositionRaw(const int joint, double *ref)
{
    CD_INFO("\n");

    //-- Check index within range
    if ( joint != 0 ) return false;

    int32_t data;

    if (!sdoClient->upload("Target position", &data, 0x607A))
    {
        return false;
    }

    *ref = data / ((encoderPulses / 360.0) * this->tr);
    return true;
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTargetPositionsRaw(double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// --------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTargetPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    CD_WARNING("Missing implementation\n");

    return true;
}
