// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------ IVelocity Related ----------------------------------------

bool teo::BodyBot::setVelocityMode() {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->setVelocityMode(j);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::velocityMove(int j, double sp) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    //*************************************************************
    uint8_t msg_vel[]={0x23,0xFF,0x60,0x00,0x00,0x00,0x00,0x00}; // Velocity target

    int sendVel = sp * (drivers[j]->getTr()) * 11.11112;  // Apply tr & convert units to encoder increments
    memcpy(msg_vel+4,&sendVel,4);

    if( ! drivers[j]->send(0x600, 8, msg_vel)){
        CD_ERROR("Sent \"velocity target\" to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"velocity target\" to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::velocityMove(const double *sp) {
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<drivers.size(); j++)
    {
        ok &= this->velocityMove(j,sp[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

