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

    // -- Save the last reference speed (double sp) for single joint (int j)
    refVelocitySemaphore.wait();
    refVelocity[j] = sp;
    refVelocitySemaphore.post();

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
    CD_INFO("\n");

    bool ok = true;
    for(int j=0; j<nodes.size(); j++)
    {
        if( joints[j] )
        {
            ok &= this->velocityMove(j,spds[j]);
        }
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getRefVelocity(const int joint, double *vel)
{
    CD_INFO("\n");

    // -- Get the last reference speed set by velocityMove for single joint (saved in double vector)
    refVelocitySemaphore.wait();
    *vel = refVelocity[joint];
    refVelocitySemaphore.post();

    return true;
}

// ------------------------------------------------------------------------------

bool teo::CanBusControlboard::getRefVelocities(double *vels)
{
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
    {
        ok &= getRefVelocity(i,&(vels[i]));
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    CD_INFO("\n");

    bool ok = true;
    for(unsigned int i=0; i<nodes.size(); i++)
    {
        if( joints[i] )
        {
            ok &= getRefVelocity(i,&(vels[i]));
        }
    }
    return ok;
}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setVelPid(int j, const yarp::dev::Pid &pid)
{

}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::setVelPids(const yarp::dev::Pid *pids)
{

}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getVelPid(int j, yarp::dev::Pid *pid)
{

}

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::getVelPids(yarp::dev::Pid *pids)
{

}
