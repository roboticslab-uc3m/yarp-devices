// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------ IEncodersTimed Related -----------------------------------------

bool teo::BodyBot::getEncodersTimed(double *encs, double *time) {
    CD_INFO("\n");

    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getEncoderTimed(int j, double *encs, double *time) {
    CD_INFO("(%d)\n",j);

    return true;
}

// -----------------------------------------------------------------------------
