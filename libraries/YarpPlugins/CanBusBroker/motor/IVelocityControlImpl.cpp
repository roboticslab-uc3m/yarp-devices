// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::velocityMove(int j, double spd)
#else
bool CanBusBroker::velocityMove(int j, double spd)
#endif
{
    CHECK_JOINT(j);
    using raw_t = yarp::dev::IVelocityControlRaw;
    return deviceMapper.mapSingleJoint<raw_t, double>(&raw_t::velocityMoveRaw, j, spd);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::velocityMove(const double * spds)
#else
bool CanBusBroker::velocityMove(const double * spds)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IVelocityControlRaw::velocityMoveRaw, spds);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::velocityMove(int n_joint, const int * joints, const double * spds)
#else
bool CanBusBroker::velocityMove(int n_joint, const int * joints, const double * spds)
#endif
{
    return deviceMapper.mapJointGroup(&yarp::dev::IVelocityControlRaw::velocityMoveRaw, n_joint, joints, spds);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTargetVelocity(int joint, double * vel)
#else
bool CanBusBroker::getRefVelocity(int joint, double * vel)
#endif
{
    CHECK_JOINT(joint);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapSingleJoint(&yarp::dev::IVelocityControlRaw::getTargetVelocityRaw, joint, vel);
#else
    return deviceMapper.mapSingleJoint(&yarp::dev::IVelocityControlRaw::getRefVelocityRaw, joint, vel);
#endif
}

// ------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTargetVelocities(double * vels)
#else
bool CanBusBroker::getRefVelocities(double * vels)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapAllJoints(&yarp::dev::IVelocityControlRaw::getTargetVelocitiesRaw, vels);
#else
    return deviceMapper.mapAllJoints(&yarp::dev::IVelocityControlRaw::getRefVelocitiesRaw, vels);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTargetVelocities(int n_joint, const int * joints, double * vels)
#else
bool CanBusBroker::getRefVelocities(int n_joint, const int * joints, double * vels)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapJointGroup(&yarp::dev::IVelocityControlRaw::getTargetVelocitiesRaw, n_joint, joints, vels);
#else
    return deviceMapper.mapJointGroup(&yarp::dev::IVelocityControlRaw::getRefVelocitiesRaw, n_joint, joints, vels);
#endif
}

// -----------------------------------------------------------------------------
