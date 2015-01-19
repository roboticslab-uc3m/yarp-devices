// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorIpos.hpp"

// ------------------ IPositionControlRaw Related ----------------------------------------

bool teo::MotorIpos::positionMoveRaw(int j, double ref) {  // encExposed = ref;
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_position_target[]={0x23,0x7A,0x60,0x00,0x00,0x00,0x00,0x00}; // Position target

    int position = ref * this->tr * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_position_target+4,&position,4);

    if( ! send( 0x600, 8, msg_position_target ) )
    {
        CD_ERROR("Could not send \"position target\". %s\n", msgToStr(0x600, 8, msg_position_target).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position target\". %s\n", msgToStr(0x600, 8, msg_position_target).c_str() );
    //*************************************************************
    //uint8_t msg_start[]={0x1F,0x00}; // Start the movement with "Discrete motion profile (change set immediately = 0)".
    uint8_t msg_start[]={0x3F,0x00}; // Start the movement with "Continuous motion profile (change set immediately = 1)".

    if( ! send( 0x200, 2, msg_start ) )
    {
        CD_ERROR("Could not send \"start position\". %s\n", msgToStr(0x200, 2, msg_start).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"start position\". %s\n", msgToStr(0x200, 2, msg_start).c_str() );
    //*************************************************************

    //-- Needed to send next. Sets "Do not assume target position" so later it accepts "Assume target position (update the new motion parameters)".
    //*************************************************************
    uint8_t msg_pos_reset[]={0x0F,0x00};  // Stop a position profile

    if( ! send( 0x200, 2, msg_pos_reset) )
    {
        CD_ERROR("Could not send \"reset position\". %s\n", msgToStr(0x200, 2, msg_pos_reset).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"reset position\". %s\n", msgToStr(0x200, 2, msg_pos_reset).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::relativeMoveRaw(int j, double delta) {
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;


    //*************************************************************
    uint8_t msg_position_target[]={0x23,0x7A,0x60,0x00,0x00,0x00,0x00,0x00}; // Position target

    int sendDelta = delta * this->tr * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_position_target+4,&sendDelta,4);

    if( ! send( 0x600, 8, msg_position_target ) )
    {
        CD_ERROR("Could not send \"position target\". %s\n", msgToStr(0x600, 8, msg_position_target).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position target\". %s\n", msgToStr(0x600, 8, msg_position_target).c_str() );
    //*************************************************************
    //uint8_t msg_start_rel[]={0x5F,0x00}; // Start the movement with "Discrete motion profile (change set immediately = 0)".
    uint8_t msg_start_rel[]={0x7F,0x00}; // Start the movement with "Continuous motion profile (change set immediately = 1)".


    if( ! send( 0x200, 2, msg_start_rel ) )
    {
        CD_ERROR("Could not send \"start rel position. %s\n", msgToStr(0x200, 2, msg_start_rel).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"start rel position\". %s\n", msgToStr(0x200, 2, msg_start_rel).c_str() );;
    //*************************************************************

    //-- Needed to send next. Sets "Do not assume target position" so later it accepts "Assume target position (update the new motion parameters)".
    //*************************************************************
    uint8_t msg_pos_reset[]={0x0F,0x00};  // Stop a position profile

    if( ! send( 0x200, 2, msg_pos_reset) )
    {
        CD_ERROR("Could not send \"reset position\". %s\n", msgToStr(0x200, 2, msg_pos_reset).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"reset position\". %s\n", msgToStr(0x200, 2, msg_pos_reset).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::checkMotionDoneRaw(int j, bool *flag) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msgStatus[] = {0x40,0x41,0x60,0x00,0x00,0x00,0x00,0x00}; //2064: Memory position
    if( ! send( 0x600, 8, msgStatus))
    {
        CD_ERROR("Could not send status query. %s\n", msgToStr(0x600, 8, msgStatus).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"msgStatus\". %s\n", msgToStr(0x600, 8, msgStatus).c_str() );

    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    Time::delay(DELAY);  //-- Wait for read update. Could implement semaphore waiting for specific message...
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    *flag = targetReached;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setRefSpeedRaw(int j, double sp) {
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************

    uint8_t msg_posmode_speed[]={0x23,0x81,0x60,0x00,0x00,0x00,0x00,0x00};

    int sendRefSpeed = sp * this->tr * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_posmode_speed+4,&sendRefSpeed,4);

    if( ! send( 0x600, 8, msg_posmode_speed) )
    {
        CD_ERROR("Could not send \"posmode_speed\". %s\n", msgToStr(0x600, 8, msg_posmode_speed).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"posmode_speed\". %s\n", msgToStr(0x600, 8, msg_posmode_speed).c_str() );
    //*************************************************************

    //-- Store new value locally as we can not retrieve it from the driver for now.
    refSpeed = sp;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setRefAccelerationRaw(int j, double acc) {
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_posmode_acc[]={0x23,0x83,0x60,0x00,0x00,0x00,0x00,0x00};

    int sendRefAcc = acc * this->tr * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_posmode_acc+4,&sendRefAcc,4);

    if( ! send( 0x600, 8, msg_posmode_acc) )
    {
        CD_ERROR("Could not send \"posmode_acc\". %s\n", msgToStr(0x600, 8, msg_posmode_acc).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"posmode_acc\". %s\n", msgToStr(0x600, 8, msg_posmode_acc).c_str() );
    //*************************************************************

    //-- Store new value locally as we can not retrieve it from the driver for now.
    refAcceleration = acc ;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getRefSpeedRaw(int j, double *ref) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = refSpeed;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getRefAccelerationRaw(int j, double *acc) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = refAcceleration;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::stopRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------
