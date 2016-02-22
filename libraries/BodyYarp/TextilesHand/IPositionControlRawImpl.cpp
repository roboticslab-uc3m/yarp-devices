// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// ------------------ IPositionControlRaw Related ----------------------------------------

bool teo::TextilesHand::positionMoveRaw(int j, double ref) {  // encExposed = ref;
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;

    unsigned char cmdByte;
    if (ref == 0)
        cmdByte = 'a';
    else if (ref == 1)
        cmdByte = 'b';
    else
        return false;
    int res = serialport_writebyte(fd, cmdByte);
    if(res==-1) return false;

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
