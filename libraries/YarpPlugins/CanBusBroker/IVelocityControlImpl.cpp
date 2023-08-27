// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;
using raw_t = yarp::dev::IVelocityControlRaw;

// -----------------------------------------------------------------------------

bool CanBusBroker::velocityMove(int j, double spd)
{
    yCTrace(CBB, "%d %f", j, spd);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, double>(&yarp::dev::IVelocityControlRaw::velocityMoveRaw, j, spd);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::velocityMove(const double * spds)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IVelocityControlRaw::velocityMoveRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::velocityMove(int n_joint, const int * joints, const double * spds)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IVelocityControlRaw::velocityMoveRaw, n_joint, joints, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefVelocity(int joint, double * vel)
{
    yCTrace(CBB, "%d", joint);
    CHECK_JOINT(joint);
    return deviceMapper.mapSingleJoint(&yarp::dev::IVelocityControlRaw::getRefVelocityRaw, joint, vel);
}

// ------------------------------------------------------------------------------

bool CanBusBroker::getRefVelocities(double * vels)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IVelocityControlRaw::getRefVelocitiesRaw, vels);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefVelocities(int n_joint, const int * joints, double * vels)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IVelocityControlRaw::getRefVelocitiesRaw, n_joint, joints, vels);
}

// -----------------------------------------------------------------------------
