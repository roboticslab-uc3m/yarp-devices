// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// ------------------- IControlLimitsRaw Related ------------------------------------

bool roboticslab::FakeJoint::setLimitsRaw(int axis, double min, double max)
{
    CD_INFO("(%d,%f,%f)\n",axis,min,max);

    //-- Check index within range
    if ( axis != 0 ) return false;

    //-- Store the new limits locally.
    this->min = min;
    this->max = max;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::getLimitsRaw(int axis, double *min, double *max)
{
    CD_INFO("(%d)\n",axis);

    //-- Check index within range
    if( axis != 0 ) return false;

    //-- Get the limits that have been locally stored.
    *min = this->min;
    *max = this->max;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::setVelLimitsRaw(int axis, double min, double max)
{
    CD_INFO("(%d,%f,%f)\n",axis,min,max);

    //-- Check index within range
    if( axis != 0 ) return false;

    //-- Update the limits that have been locally stored.
    this->minVel = min;
    this->maxVel = max;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::getVelLimitsRaw(int axis, double *min, double *max)
{
    CD_INFO("(%d)\n",axis);

    //-- Check index within range
    if( axis != 0 ) return false;

    //-- Get the limits that have been locally stored.
    *min = this->minVel;
    *max = this->maxVel;

    return true;
}

// -----------------------------------------------------------------------------
