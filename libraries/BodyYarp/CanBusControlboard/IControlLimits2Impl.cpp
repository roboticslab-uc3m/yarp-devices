// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------- IControlLimits Related ------------------------------------

bool teo::CanBusControlboard::setLimits(int axis, double min, double max)
{
    CD_INFO("(%d,%f,%f)\n",axis,min,max);

    //-- Check index within range
    if ( ! this->indexWithinRange(axis) ) return false;

    return iControlLimits2Raw[axis]->setLimitsRaw( 0, min, max );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getLimits(int axis, double *min, double *max)
{
    CD_INFO("(%d)\n",axis);

    //-- Check index within range
    if( axis >= nodes.size() ) return false;

    return iControlLimits2Raw[axis]->getLimitsRaw( 0, min, max );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setVelLimits(int axis, double min, double max)
{
    return iControlLimits2Raw[axis]->setVelLimitsRaw( 0, min, max );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getVelLimits(int axis, double *min, double *max)
{
    return iControlLimits2Raw[axis]->getVelLimitsRaw( 0, min, max );
}

// -----------------------------------------------------------------------------
