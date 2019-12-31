// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;
using raw_t = yarp::dev::IVelocityControlRaw;

// -----------------------------------------------------------------------------

bool CanBusControlboard::velocityMove(int j, double spd)
{
    CD_DEBUG("(%d, %f)\n", j, spd);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, double>(&yarp::dev::IVelocityControlRaw::velocityMoveRaw, j, spd);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::velocityMove(const double * spds)
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::IVelocityControlRaw::velocityMoveRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::velocityMove(int n_joint, const int * joints, const double * spds)
{
    CD_DEBUG("\n");
    return deviceMapper.mapJointGroup(&yarp::dev::IVelocityControlRaw::velocityMoveRaw, n_joint, joints, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefVelocity(int joint, double * vel)
{
    CD_DEBUG("%d\n", joint);
    CHECK_JOINT(joint);
    return deviceMapper.mapSingleJoint(&yarp::dev::IVelocityControlRaw::getRefVelocityRaw, joint, vel);
}

// ------------------------------------------------------------------------------

bool CanBusControlboard::getRefVelocities(double * vels)
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::IVelocityControlRaw::getRefVelocitiesRaw, vels);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefVelocities(int n_joint, const int * joints, double * vels)
{
    CD_DEBUG("\n");
    return deviceMapper.mapJointGroup(&yarp::dev::IVelocityControlRaw::getRefVelocitiesRaw, n_joint, joints, vels);
}

// -----------------------------------------------------------------------------
