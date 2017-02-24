// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------ IVelocityControl Related ----------------------------------------

bool teo::CanBusControlboard::setVelocityMode()
{
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->setVelocityMode(j);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::velocityMove(int j, double sp)
{
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    return iVelocityControlRaw[j]->velocityMoveRaw( 0, sp );
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::velocityMove(const double *sp)
{
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        ok &= this->velocityMove(j,sp[j]);
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::velocityMove(const int n_joint, const int *joints, const double *spds)
{

}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getRefVelocity(const int joint, double *vel)
{

}

// ------------------------------------------------------------------------------

bool getRefVelocities(double *vels)
{

}

// -----------------------------------------------------------------------------

bool getRefVelocities(const int n_joint, const int *joints, double *vels)
{

}

// -----------------------------------------------------------------------------

bool setVelPid(int j, const yarp::dev::Pid &pid)
{

}

// -----------------------------------------------------------------------------

bool setVelPids(const yarp::dev::Pid *pids)
{

}

// -----------------------------------------------------------------------------

bool getVelPid(int j, yarp::dev::Pid *pid)
{

}

// -----------------------------------------------------------------------------

bool getVelPids(yarp::dev::Pid *pids)
{

}
