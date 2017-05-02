// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

// ######################### IVelocityControlRaw Related ###########################

bool teo::LacqueyFetch::velocityMoveRaw(int j, double sp)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::velocityMoveRaw(const double *sp)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ######################### IVelocityControl2Raw Related #########################

bool teo::LacqueyFetch::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::getRefVelocityRaw(const int joint, double *vel)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// ------------------------------------------------------------------------------

bool teo::LacqueyFetch::getRefVelocitiesRaw(double *vels)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------
/*
bool teo::LacqueyFetch::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    CD_ERROR("\n");
    return false;
}
*/
// -----------------------------------------------------------------------------
/*
bool teo::LacqueyFetch::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    CD_ERROR("\n");
    return false;
}
*/
// -----------------------------------------------------------------------------
/*
bool teo::LacqueyFetch::stopRaw(const int n_joint, const int *joints)
{
    CD_ERROR("\n");
    return false;
}
*/
// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::setVelPidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::setVelPidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::getVelPidRaw(int j, yarp::dev::Pid *pid)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::getVelPidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("Missing implementation\n");
    return false;
}
