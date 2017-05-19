// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------- IControlMode Related ------------------------------------

bool teo::CanBusControlboard::setPositionMode(int j)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iControlMode2Raw[j]->setPositionModeRaw( 0 );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setVelocityMode(int j)
{
    CD_DEBUG("(%d)\n",j);

    return iControlMode2Raw[j]->setVelocityModeRaw( 0 );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setTorqueMode(int j)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iControlMode2Raw[j]->setTorqueModeRaw( 0 );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setImpedancePositionMode(int j)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iControlMode2Raw[j]->setImpedancePositionModeRaw( 0 );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setImpedanceVelocityMode(int j)
{
    CD_DEBUG("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iControlMode2Raw[j]->setImpedanceVelocityModeRaw( 0 );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setOpenLoopMode(int j)
{
    CD_ERROR("(%d)\n",j);  //-- Removed in YARP 2.3.70
    return false;
}

// ---------------------- IControlMode2 Related  ---------------------------------

bool teo::CanBusControlboard::getControlMode(int j, int *mode)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iControlMode2Raw[j]->getControlModeRaw( 0, mode );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getControlModes(int *modes)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i < nodes.size(); i++)
        ok &= getControlMode(i,&modes[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i < n_joint; i++)
        ok &= getControlMode(joints[i],&modes[i]);
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setControlMode(const int j, const int mode)
{
    CD_DEBUG("(%d, %d)\n",j,mode);

    bool ok = true;
    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    if( mode == VOCAB_CM_POSITION )
        ok = setPositionMode(j);
    if( mode == VOCAB_CM_VELOCITY )
        ok = setVelocityMode(j);
    if( mode == VOCAB_CM_TORQUE )
        ok = setTorqueMode(j);
    if( mode == VOCAB_CM_IMPEDANCE_POS )
        ok = setImpedancePositionMode(j);
    if( mode == VOCAB_CM_IMPEDANCE_VEL )
        ok = setImpedanceVelocityMode(j);
    /*if( mode == VOCAB_CM_OPENLOOP )
        ok = setOpenLoopMode(j);*/

    return ok;

}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setControlModes(const int n_joint, const int *joints, int *modes)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(int j=0; j<n_joint; j++)
    {                
        ok &= this->setControlMode(joints[j],modes[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setControlModes(int *modes)
{
    CD_DEBUG("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
        ok &= setControlMode(i,modes[i]);
    return ok;
}
