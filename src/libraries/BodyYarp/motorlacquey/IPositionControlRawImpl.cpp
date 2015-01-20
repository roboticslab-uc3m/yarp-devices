// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorLacquey.hpp"

// ------------------ IPositionControlRaw Related ----------------------------------------

bool teo::MotorLacquey::positionMoveRaw(int j, double ref) {  // encExposed = ref;
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

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::relativeMoveRaw(int j, double delta) {
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::checkMotionDoneRaw(int j, bool *flag) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::setRefSpeedRaw(int j, double sp) {
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::setRefAccelerationRaw(int j, double acc) {
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::getRefSpeedRaw(int j, double *ref) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::getRefAccelerationRaw(int j, double *acc) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::MotorLacquey::stopRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet.\n");

    return true;
}

// -----------------------------------------------------------------------------
