// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <ColorDebug.hpp>

// ------------------ IVelocity2 Related ----------------------------------------

bool roboticslab::FakeControlboard::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    CD_DEBUG("\n");
    // must implement mask!
    return velocityMove(spds);
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getRefVelocity(const int joint, double *vel)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getRefVelocities(double *vels)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_MAJOR != 3
bool roboticslab::FakeControlboard::setVelPid(int j, const yarp::dev::Pid &pid)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setVelPids(const yarp::dev::Pid *pids)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getVelPid(int j, yarp::dev::Pid *pid)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getVelPids(yarp::dev::Pid *pids)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------
#endif // YARP_VERSION_MAJOR != 3
