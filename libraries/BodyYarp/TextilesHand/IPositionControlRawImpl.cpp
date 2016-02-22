// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// ------------------ IPositionControlRaw Related ----------------------------------------

bool teo::TextilesHand::positionMoveRaw(int j, double ref) {  // encExposed = ref;
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;

    //Adjust range:
    if (ref > 1023){
        ref=1023;
    }
    else if (ref < -1023){
        ref=-1023;
    }

    //*************************************************************
    uint8_t msg_position_target[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // Position target
    uint16_t pwm;
    //Send appropriate message:
    if (ref==0) {
        msg_position_target[0]=0xF0;
    } else if (ref>0) {
        pwm=ref;
        msg_position_target[1]=pwm>>2;
        msg_position_target[0]=0xA0 + (pwm & 0b0000000000000011);
    } else {
        pwm=abs(ref);
        msg_position_target[1]=pwm>>2;
        msg_position_target[0]=0xC0 + (pwm & 0b0000000000000011);
    }

    if( ! send( 0x600, 2, msg_position_target ) )
    {
        CD_ERROR("Could not send \"position target8\". %s\n", msgToStr(0x600, 2, msg_position_target).c_str() );
        return false;
    }
    CD_SUCCESS("Sent \"position target8\". %s\n", msgToStr(0x600, 2, msg_position_target).c_str() );
    //*************************************************************

    encoderReady.wait();
    this->encoder = ref;  // Already passed through Adjust range.
    encoderReady.post();

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::relativeMoveRaw(int j, double delta) {
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TextilesHand).\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::checkMotionDoneRaw(int j, bool *flag) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *flag = true;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::setRefSpeedRaw(int j, double sp) {
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::setRefAccelerationRaw(int j, double acc) {
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::getRefSpeedRaw(int j, double *ref) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::getRefAccelerationRaw(int j, double *acc) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::stopRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------
