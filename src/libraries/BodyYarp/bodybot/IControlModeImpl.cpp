// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------- IControlMode Related ------------------------------------

bool teo::BodyBot::setPositionMode(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    //*************************************************************
    uint8_t msg_position_mode[] = {0x2F,0x60,0x60,0x00,0x01};  // Position mode

    if( ! drivers[j]->send( 0x600, 5, msg_position_mode) )
    {
        CD_ERROR("Could not send \"position_mode\" to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"position_mode\" to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setVelocityMode(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    //*************************************************************
    uint8_t msg_velocity_mode[]={0x2F,0x60,0x60,0x00,0x03}; // Velocity mode

    if( ! drivers[j]->send( 0x600, 5, msg_velocity_mode) )
    {
        CD_ERROR("Could not send \"vel_mode\" to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"vel_mode\" to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setTorqueMode(int j)  {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    //*************************************************************
    //-- External reference type. Slave receives reference through CAN (manual 208 of 263).
    uint8_t msg_ref_type[]={0x2B,0x1D,0x20,0x00,0x01,0x00,0x00,0x00};  //CAN

    if( ! drivers[j]->send( 0x600, 8, msg_ref_type) )
    {
        CD_ERROR("Could not send \"ref_type\" to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"ref_type\" to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************
    //-- Mode -5 (manual 209 of 263).
    uint8_t msg_mode_torque[]={0x2F,0x60,0x60,0x00,0xFB,0x00,0x00,0x00};

    if( ! drivers[j]->send( 0x600, 8, msg_mode_torque) )
    {
        CD_ERROR("Could not send \"mode_torque\" to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"mode_torque\" to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************
    //-- Control word (manual 215 of 263).
    uint8_t msg_torque_word[] = {0x1F,0x00};

    if( ! drivers[j]->send( 0x200, 2, msg_torque_word) )
    {
        CD_ERROR("Could not send msg_torque_word to canId: %d.\n",drivers[j]->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"torque_word\" to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setImpedancePositionMode(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setImpedanceVelocityMode(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setOpenLoopMode(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getControlMode(int j, int *mode) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    this->checkStatus(j);

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getControlModes(int *modes) {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i < drivers.size(); i++)
        ok &= getControlMode(i,&modes[i]);
    return ok;
}

// -----------------------------------------------------------------------------
