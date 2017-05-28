// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------ IVelocityControl2 related ----------------------------------------

bool roboticslab::AmorControlboard::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    CD_DEBUG("(%d)\n", n_joint);
    if (!indexWithinRange(n_joint))
        return false;
    // must implement mask!
    return velocityMove(spds);
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefVelocity(const int joint, double *vel)
{
    CD_DEBUG("(%d)\n", joint);
    if (!indexWithinRange(joint))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefVelocities(double *vels)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    CD_DEBUG("(%d)\n", n_joint);
    if (!indexWithinRange(n_joint))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setVelPid(int j, const yarp::dev::Pid &pid)
{
    CD_DEBUG("(%d)\n", j);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setVelPids(const yarp::dev::Pid *pids)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getVelPid(int j, yarp::dev::Pid *pid)
{
    CD_DEBUG("(%d)\n", j);
    if (!indexWithinRange(j))
        return false;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getVelPids(yarp::dev::Pid *pids)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------
