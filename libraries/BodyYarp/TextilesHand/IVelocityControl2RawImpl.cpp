// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

//  ########################### IVelocityControlRaw implementations ###########################

bool teo::TextilesHand::velocityMoveRaw(int j, double sp)
{
    CD_INFO("Missing implementation\n");
    return true;
}

// --------------------------------------------------------------------------------------------

bool teo::TextilesHand::velocityMoveRaw(const double *sp)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//  ########################### IVelocityControl2Raw implementations ###########################

bool teo::TextilesHand::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::TextilesHand::getRefVelocityRaw(const int joint, double *vel)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::TextilesHand::getRefVelocitiesRaw(double *vels)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::TextilesHand::getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::TextilesHand::setVelPidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::TextilesHand::setVelPidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::TextilesHand::getVelPidRaw(int j, yarp::dev::Pid *pid)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool teo::TextilesHand::getVelPidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("Missing implementation\n");
    return false;
}
