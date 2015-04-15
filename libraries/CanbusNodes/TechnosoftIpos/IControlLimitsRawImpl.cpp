// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------- IControlLimitsRaw Related ------------------------------------

bool teo::TechnosoftIpos::setLimitsRaw(int axis, double min, double max) {
    CD_INFO("(%d,%f,%f)\n",axis,min,max);

    //-- Check index within range
    if ( axis != 0 ) return false;

    //*************************************************************
    uint8_t msg_position_min[]={0x23,0x7D,0x60,0x01,0x00,0x00,0x00,0x00}; // 0x01 is subindex 1, Manual 607Dh: Software position limit

    int sendMin = min * this->tr * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_position_min+4,&sendMin,4);

    if( ! send( 0x600, 8, msg_position_min ) )
    {
        CD_ERROR("Could not send position min. %s\n", msgToStr(0x600, 8, msg_position_min).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position min\". %s\n", msgToStr(0x600, 8, msg_position_min).c_str() );
    //*************************************************************
    uint8_t msg_position_max[]={0x23,0x7D,0x60,0x02,0x00,0x00,0x00,0x00}; // 0x02 is subindex 2, Manual 607Dh: Software position limit

    int sendMax = max * this->tr * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_position_max+4,&sendMax,4);

    if( ! send( 0x600, 8, msg_position_max ) )
    {
        CD_ERROR("Could not send position max. %s\n", msgToStr(0x600, 8, msg_position_max).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position max\". %s\n", msgToStr(0x600, 8, msg_position_max).c_str() );
    //*************************************************************

    //-- Store the new limits locally.
    this->max;
    this->min;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::getLimitsRaw(int axis, double *min, double *max) {
    CD_INFO("(%d)\n",axis);

    //-- Check index within range
    if( axis != 0 ) return false;

    //-- Get the limits that have been locally stored.
    *min = this->min;
    *max = this->max;

    return true;
}

// -----------------------------------------------------------------------------

