// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IVelocityControlRaw Related ----------------------------------------

bool teo::TechnosoftIpos::velocityMoveRaw(int j, double sp)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    //*************************************************************
    uint8_t msg_vel[]= {0x23,0xFF,0x60,0x00,0x00,0x00,0x00,0x00}; // Velocity target

    //uint16_t sendRefSpeed = sp * this->tr * 0.01138;  // Appply tr & convert units to encoder increments
    //memcpy(msg_posmode_speed+6,&sendRefSpeed,2);
    //float sendRefSpeed = sp * this->tr / 22.5;  // Apply tr & convert units to encoder increments
    //int32_t sendRefSpeedFormated = roundf(sendRefSpeed * 65536);  // 65536 = 2^16

    //-- 65536 for FIXED32
    //-- 0.01138 = ( 4 * 1024 pulse / 360 deg ) * (0.001 s / sample)   // deg/s -> pulse/sample  = UI (vel)
    int32_t sendRefSpeedFormated = roundf( sp * this->tr * 745.8 ); //-- 65536 * 0.01138 = 745.8
    memcpy(msg_vel+4,&sendRefSpeedFormated,4);

    if( ! send(0x600, 8, msg_vel))
    {
        CD_ERROR("Sent \"velocity target\". %s\n", msgToStr(0x600, 8, msg_vel).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"velocity target\". %s\n", msgToStr(0x600, 8, msg_vel).c_str() );
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------
