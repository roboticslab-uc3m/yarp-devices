// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------ IPositionDirect Related ----------------------------------

bool teo::BodyBot::setPositionDirectMode() {
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i < drivers.size(); i++)
        ok &= iPositionDirectRaw[i]->setPositionDirectModeRaw();  // No existing single mode.

    Time::delay(1);  //-- Seems like a "must".

    return ok;
}


// -----------------------------------------------------------------------------

bool teo::BodyBot::setPosition(int j, double ref) {
    CD_INFO("\n");

    return iPositionDirectRaw[j]->setPositionRaw( 0, ref );
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setPositions(const int n_joint, const int *joints, double *refs) {
    CD_INFO("n_joint:%d, drivers.size():" CD_SIZE_T "\n",n_joint,drivers.size());

    bool ok = true;
    for(unsigned int i=0; i < drivers.size(); i++)
        ok &= iPositionDirectRaw[i]->setPositionsRaw(n_joint, joints, refs); // No existing single mode.
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::setPositions(const double *refs) {
    bool ok = true;
    for(unsigned int i=0; i < drivers.size(); i++)
        ok &= iPositionDirectRaw[i]->setPositionsRaw(refs); // No existing single mode.
    return ok;
}

// -----------------------------------------------------------------------------
