// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

//  ########################### IVelocityControlRaw implementations ###########################

bool teo::CuiAbsolute::velocityMoveRaw(int j, double sp)
{
    CD_INFO("Missing implementation\n");
    return true;
}

// --------------------------------------------------------------------------------------------

bool teo::CuiAbsolute::velocityMoveRaw(const double *sp)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//  ########################### IVelocityControl2Raw implementations ###########################

bool teo::CuiAbsolute::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::CuiAbsolute::getRefVelocityRaw(const int joint, double *vel)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::CuiAbsolute::getRefVelocitiesRaw(double *vels)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::CuiAbsolute::getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::CuiAbsolute::setVelPidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::CuiAbsolute::setVelPidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::CuiAbsolute::getVelPidRaw(int j, yarp::dev::Pid *pid)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::CuiAbsolute::getVelPidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("Missing implementation\n");
    return false;
}
