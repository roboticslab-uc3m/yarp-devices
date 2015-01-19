// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------- IControlMode Related ------------------------------------

bool teo::BodyBot::setPositionMode(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iControlModeRaw[j]->setPositionModeRaw( 0 );
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setVelocityMode(int j) {
    CD_INFO("(%d)\n",j);

    return iControlModeRaw[j]->setVelocityModeRaw( 0 );
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setTorqueMode(int j)  {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iControlModeRaw[j]->setTorqueModeRaw( 0 );
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setImpedancePositionMode(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iControlModeRaw[j]->setImpedancePositionModeRaw( 0 );
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setImpedanceVelocityMode(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iControlModeRaw[j]->setImpedanceVelocityModeRaw( 0 );
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setOpenLoopMode(int j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iControlModeRaw[j]->setOpenLoopModeRaw( 0 );
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getControlMode(int j, int *mode) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iControlModeRaw[j]->getControlModeRaw( 0, mode );
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
