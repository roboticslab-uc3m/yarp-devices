// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorIpos.hpp"

// ------------------- IControlModeRaw Related ------------------------------------

bool teo::MotorIpos::setPositionModeRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_position_mode[] = {0x2F,0x60,0x60,0x00,0x01};  // Position mode

    if( ! send( 0x600, 5, msg_position_mode) )
    {
        CD_ERROR("Could not send \"position_mode\". %s\n", msgToStr(0x600, 5, msg_position_mode).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position_mode\". %s\n", msgToStr(0x600, 5, msg_position_mode).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setVelocityModeRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_velocity_mode[]={0x2F,0x60,0x60,0x00,0x03}; // Velocity mode

    if( ! send( 0x600, 5, msg_velocity_mode) )
    {
        CD_ERROR("Could not send \"vel_mode\". %s\n", msgToStr(0x600, 5, msg_velocity_mode).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"vel_mode\". %s\n", msgToStr(0x600, 5, msg_velocity_mode).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setTorqueModeRaw(int j)  {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    //-- External reference type. Slave receives reference through CAN (manual 208 of 263).
    uint8_t msg_ref_type[]={0x2B,0x1D,0x20,0x00,0x01,0x00,0x00,0x00};  //CAN

    if( ! send( 0x600, 8, msg_ref_type) )
    {
        CD_ERROR("Could not send \"ref_type\". %s\n", msgToStr(0x600, 8, msg_ref_type).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"ref_type\". %s\n", msgToStr(0x600, 8, msg_ref_type).c_str() );
    //*************************************************************
    //-- Mode -5 (manual 209 of 263).
    uint8_t msg_mode_torque[]={0x2F,0x60,0x60,0x00,0xFB,0x00,0x00,0x00};

    if( ! send( 0x600, 8, msg_mode_torque) )
    {
        CD_ERROR("Could not send \"mode_torque\". %s\n", msgToStr(0x600, 8, msg_mode_torque).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"mode_torque\". %s\n", msgToStr(0x600, 8, msg_mode_torque).c_str() );
    //*************************************************************
    //-- Control word (manual 215 of 263).
    uint8_t msg_torque_word[] = {0x1F,0x00};

    if( ! send( 0x200, 2, msg_torque_word) )
    {
        CD_ERROR("Could not send msg_torque_word. %s\n", msgToStr(0x200, 2, msg_torque_word).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"torque_word\". %s\n", msgToStr(0x200, 2, msg_torque_word).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setImpedancePositionModeRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (motoripos).\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setImpedanceVelocityModeRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (motoripos).\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::setOpenLoopModeRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (motoripos).\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorIpos::getControlModeRaw(int j, int *mode) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msgOperationDisplay[] = {0x40,0x61,0x60,0x00,0x00,0x00,0x00,0x00}; // Manual 6061h: Modes of Operation display
    if( ! send( 0x600, 8, msgOperationDisplay))
    {
        CD_ERROR("Could not send modes of operation display. %s\n", msgToStr(0x600, 8, msgOperationDisplay).c_str() );
        return false;
    }
    CD_SUCCESS("Sent modes of operation display. %s\n", msgToStr(0x600, 8, msgOperationDisplay).c_str() );
    //*************************************************************

    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    Time::delay(DELAY);  // Must delay as it will be from same driver.
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    getModeReady.wait();
    *mode = getMode;
    getModeReady.post();

    //-- Ya de paso...
    //*************************************************************
    uint8_t msgStatus[] = {0x40,0x41,0x60,0x00,0x00,0x00,0x00,0x00}; // Manual 6041h: Status word
    if( ! send( 0x600, 8, msgStatus))
    {
        CD_ERROR("Could not send status query. %s\n", msgToStr(0x600, 8, msgStatus).c_str() );
        return false;
    }
    CD_SUCCESS("Sent status query. %s\n", msgToStr(0x600, 8, msgStatus).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------
