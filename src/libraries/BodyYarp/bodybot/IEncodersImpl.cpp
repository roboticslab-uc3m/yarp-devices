// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------ IEncoders Related -----------------------------------------

bool teo::BodyBot::resetEncoder(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return this->setEncoder(j,0);
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::resetEncoders() {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i < drivers.size(); i++)
        ok &= resetEncoder(i);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setEncoder(int j, double val) {  // encExposed = val;
    CD_INFO("(%d,%f)\n",j,val);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    //*************************************************************
    uint8_t msg_setEncoder[]={0x23,0x81,0x20,0x00,0x00,0x00,0x00,0x00}; // "Set/Change the actual motor position"

    int sendEnc = val * (drivers[j]->getTr()) * 11.11112;  // Apply tr & convert units to encoder increments
    memcpy(msg_setEncoder+4,&sendEnc,4);

    if( ! drivers[j]->send(0x600, 8, msg_setEncoder)){
        CD_ERROR("Sent \"set encoder\" to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"set encoder\" to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setEncoders(const double *vals) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i < drivers.size(); i++)
        ok &= setEncoder(i,vals[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getEncoder(int j, double *v) {
    //CD_INFO("%d\n",j);  //-- Too verbose in stream.

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    //*************************************************************
    uint8_t msg_read[]={0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00}; // Query position.
    if( ! drivers[j]->send( 0x600, 8, msg_read) )
    {
        CD_ERROR("Could not send read.\n");
        return false;
    }
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    Time::delay(DELAY);  // Must delay as it will be from same driver.
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    *v = drivers[j]->getEncoder();
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getEncoders(double *encs) {
    //CD_INFO("\n");  //-- Too verbose in stream.

    uint8_t msg_read[]={0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00}; // Query position.

    bool ok = true;
    //*************************************************************
    for(unsigned int j=0; j < drivers.size(); j++)
    {
        if( ! drivers[j]->send( 0x600, 8, msg_read) )
        {
            CD_ERROR("Could not send read.\n");
            return false;
        }
    }
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    Time::delay(0.002);  // Okay 0.002 for 6 drivers.
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    for(unsigned int j=0; j < drivers.size(); j++)
    {
        encs[j] = drivers[j]->getEncoder();
    }
    //*************************************************************

    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getEncoderSpeed(int j, double *sp) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getEncoderSpeeds(double *spds) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0;i<drivers.size();i++)
        ok &= getEncoderSpeed(i,&spds[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getEncoderAcceleration(int j, double *spds) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getEncoderAccelerations(double *accs) {
    CD_INFO("\n");

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

