// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------- IControlLimits Related ------------------------------------

bool teo::BodyBot::setLimits(int axis, double min, double max) {
    CD_INFO("(%d,%f,%f)\n",axis,min,max);

    //-- Check index within range
    if ( ! this->indexWithinRange(axis) ) return false;

    //*************************************************************
    uint8_t msg_position_min[]={0x23,0x7D,0x60,0x01,0x00,0x00,0x00,0x00}; // 0x01 is subindex 1

    int sendMin = min * (drivers[axis]->getTr()) * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_position_min+4,&sendMin,4);

    if( ! drivers[axis]->send( 0x600, 8, msg_position_min ) )
    {
        CD_ERROR("Could not send position min.\n");
        return false;
    }
    CD_SUCCESS("Sent \"position min\".\n");
    //*************************************************************
    uint8_t msg_position_max[]={0x23,0x7D,0x60,0x02,0x00,0x00,0x00,0x00}; // 0x02 is subindex 2

    int sendMax = max * (drivers[axis]->getTr()) * 11.11112;  // Appply tr & convert units to encoder increments
    memcpy(msg_position_max+4,&sendMax,4);

    if( ! drivers[axis]->send( 0x600, 8, msg_position_max ) )
    {
        CD_ERROR("Could not send position max.\n");
        return false;
    }
    CD_SUCCESS("Sent \"position max\".\n");
    //*************************************************************

    //-- Store the new limits locally.
    drivers[axis]->setMax(max);
    drivers[axis]->setMin(min);

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getLimits(int axis, double *min, double *max) {
    CD_INFO("(%d)\n",axis);

    //-- Check index within range
    if( axis >= drivers.size() ) return false;

    //-- Get the limits that have been locally stored.
    *min = drivers[axis]->getMin();
    *max = drivers[axis]->getMax();

    return true;
}

// -----------------------------------------------------------------------------

