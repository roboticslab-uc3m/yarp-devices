// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------ IVelocityControl Related ----------------------------------------

bool roboticslab::CanBusControlboard::velocityMove(int j, double sp)
{
    CD_DEBUG("(%d), (%f)\n",j , sp);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iVelocityControlRaw[j]->velocityMoveRaw( 0, sp );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::velocityMove(const double *sp)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->velocityMove(j,sp[j]);
    }
    return ok;
}

// ----------------------------  IVelocityControl2 Related  --------------------

bool roboticslab::CanBusControlboard::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<n_joint; j++)
    {
        ok &= this->velocityMove(joints[j],spds[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefVelocity(const int joint, double *vel)
{
    CD_DEBUG("%d\n",joint);

    //-- Check index within range
    if ( ! this->indexWithinRange(joint) ) return false;

    // -- Get the last reference speed set by velocityMove (see IVelocityControl2RawImpl.cpp contained in TechnosoftIpos ) for single joint
    return iVelocityControlRaw[joint]->getRefVelocityRaw(0, vel); // -- It can be... getRefVelocityRaw(joint, vel)
}

// ------------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefVelocities(double *vels)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
    {
        ok &= getRefVelocity(i,&(vels[i]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<n_joint; i++)
    {
        ok &= getRefVelocity(joints[i],&(vels[i]));
    }
    return ok;
}

// -----------------------------------------------------------------------------
