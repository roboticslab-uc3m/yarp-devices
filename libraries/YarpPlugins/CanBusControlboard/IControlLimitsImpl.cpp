// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------- IControlLimits Related ------------------------------------

bool roboticslab::CanBusControlboard::setLimits(int axis, double min, double max)
{
    CD_DEBUG("(%d,%f,%f)\n",axis,min,max);

    //-- Check index within range
    if ( ! this->indexWithinRange(axis) ) return false;

    return iControlLimitsRaw[axis]->setLimitsRaw( 0, min, max );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getLimits(int axis, double *min, double *max)
{
    CD_DEBUG("(%d)\n",axis);

    //-- Check index within range
    if( axis >= nodes.size() ) return false;

    return iControlLimitsRaw[axis]->getLimitsRaw( 0, min, max );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setVelLimits(int axis, double min, double max)
{
    return iControlLimitsRaw[axis]->setVelLimitsRaw( 0, min, max ); // May segfault in future if not impl?
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getVelLimits(int axis, double *min, double *max)
{
    return iControlLimitsRaw[axis]->getVelLimitsRaw( 0, min, max );  // May segfault in future if not impl?
}

// -----------------------------------------------------------------------------
