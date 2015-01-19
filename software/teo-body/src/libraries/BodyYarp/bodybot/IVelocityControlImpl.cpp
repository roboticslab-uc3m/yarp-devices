// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------ IVelocityControl Related ----------------------------------------

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

    return drivers[j]->velocityMoveRaw( 0, sp );
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

