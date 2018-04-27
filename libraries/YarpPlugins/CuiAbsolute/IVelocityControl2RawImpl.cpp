// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

//  ########################### IVelocityControlRaw implementations ###########################

bool roboticslab::CuiAbsolute::velocityMoveRaw(int j, double sp)
{
    CD_INFO("Missing implementation\n");
    return true;
}

// --------------------------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::velocityMoveRaw(const double *sp)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//  ########################### IVelocityControl2Raw implementations ###########################

bool roboticslab::CuiAbsolute::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getRefVelocityRaw(const int joint, double *vel)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getRefVelocitiesRaw(double *vels)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

#if YARP_VERSION_MAJOR != 3
bool roboticslab::CuiAbsolute::setVelPidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setVelPidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getVelPidRaw(int j, yarp::dev::Pid *pid)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getVelPidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("Missing implementation\n");
    return false;
}
#endif // YARP_VERSION_MAJOR != 3
