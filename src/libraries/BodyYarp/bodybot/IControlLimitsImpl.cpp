// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------- IControlLimits Related ------------------------------------

bool teo::BodyBot::setLimits(int axis, double min, double max) {
    CD_INFO("(%d,%f,%f)\n",axis,min,max);

    //-- Check index within range
    if ( ! this->indexWithinRange(axis) ) return false;

    return iControlLimitsRaw[axis]->setLimitsRaw( 0, min, max );
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::getLimits(int axis, double *min, double *max) {
    CD_INFO("(%d)\n",axis);

    //-- Check index within range
    if( axis >= drivers.size() ) return false;

    return iControlLimitsRaw[axis]->getLimitsRaw( 0, min, max );
}

// -----------------------------------------------------------------------------

