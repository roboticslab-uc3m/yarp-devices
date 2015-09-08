// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IVelocityControlRaw Related ----------------------------------------

bool teo::TechnosoftIpos::velocityMoveRaw(int j, double sp) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_vel[]={0x23,0xFF,0x60,0x00,0x00,0x00,0x00,0x00}; // Velocity target

    int16_t sendVel = sp * this->tr / 22.5;  // Apply tr & convert units to encoder increments
    memcpy(msg_vel+6,&sendVel,2);

    if( ! send(0x600, 8, msg_vel)){
        CD_ERROR("Sent \"velocity target\". %s\n", msgToStr(0x600, 8, msg_vel).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"velocity target\". %s\n", msgToStr(0x600, 8, msg_vel).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------
