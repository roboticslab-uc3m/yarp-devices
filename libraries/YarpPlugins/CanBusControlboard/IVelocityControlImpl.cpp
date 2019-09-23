// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::velocityMove(int j, double spd)
{
    CD_DEBUG("(%d, %f)\n", j, spd);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, spd, &yarp::dev::IVelocityControlRaw::velocityMoveRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::velocityMove(const double * spds)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(spds, &yarp::dev::IVelocityControlRaw::velocityMoveRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::velocityMove(int n_joint, const int * joints, const double * spds)
{
    CD_DEBUG("\n");
    return deviceMapper.multiJointMapping(n_joint, joints, spds, &yarp::dev::IVelocityControlRaw::velocityMoveRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefVelocity(int joint, double * vel)
{
    CD_DEBUG("%d\n", joint);
    CHECK_JOINT(joint);
    return deviceMapper.singleJointMapping(joint, vel, &yarp::dev::IVelocityControlRaw::getRefVelocityRaw);
}

// ------------------------------------------------------------------------------

bool CanBusControlboard::getRefVelocities(double * vels)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(vels, &yarp::dev::IVelocityControlRaw::getRefVelocitiesRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefVelocities(int n_joint, const int * joints, double * vels)
{
    CD_DEBUG("\n");
    return deviceMapper.multiJointMapping(n_joint, joints, vels, &yarp::dev::IVelocityControlRaw::getRefVelocitiesRaw);
}

// -----------------------------------------------------------------------------
