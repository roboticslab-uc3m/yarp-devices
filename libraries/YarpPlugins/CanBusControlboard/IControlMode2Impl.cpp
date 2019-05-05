// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------- IControlMode Related ------------------------------------

bool roboticslab::CanBusControlboard::getControlMode(int j, int *mode)
{
    //CD_DEBUG("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iControlMode2Raw[j]->getControlModeRaw( 0, mode );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getControlModes(int *modes)
{
    //CD_DEBUG("\n");  //-- Too verbose in controlboardwrapper2 stream

    bool ok = true;
    for(unsigned int i=0; i < nodes.size(); i++)
        ok &= getControlMode(i,&modes[i]);
    return ok;
}

// ---------------------- IControlMode2 Related  ---------------------------------

bool roboticslab::CanBusControlboard::getControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i < n_joint; i++)
        ok &= getControlMode(joints[i],&modes[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setControlMode(const int j, const int mode)
{
    CD_DEBUG("(%d, %d)\n",j,mode);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iControlMode2Raw[j]->setControlModeRaw( 0, mode );
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("(%d)\n",n_joint);

    bool ok = true;
    for(int j=0; j<n_joint; j++)
    {
        ok &= this->setControlMode(joints[j],modes[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setControlModes(int *modes)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
        ok &= setControlMode(i,modes[i]);
    return ok;
}
