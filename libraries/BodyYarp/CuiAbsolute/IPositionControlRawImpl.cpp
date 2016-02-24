// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

// ------------------ IPositionControlRaw Related ----------------------------------------

bool teo::CuiAbsolute::positionMoveRaw(int j, double ref) {  // encExposed = ref;
    CD_INFO("(%d,%f)\n",j,ref);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (CuiAbsolute).\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::relativeMoveRaw(int j, double delta) {
    CD_INFO("(%d, %f)\n",j,delta);

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (CuiAbsolute).\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::checkMotionDoneRaw(int j, bool *flag) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *flag = true;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::setRefSpeedRaw(int j, double sp) {
    CD_INFO("(%d, %f)\n",j,sp);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::setRefAccelerationRaw(int j, double acc) {
    CD_INFO("(%d, %f)\n",j,acc);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::getRefSpeedRaw(int j, double *ref) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *ref = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::getRefAccelerationRaw(int j, double *acc) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    *acc = 0;

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CuiAbsolute::stopRaw(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( j != 0 ) return false;

    return true;
}

// -----------------------------------------------------------------------------
