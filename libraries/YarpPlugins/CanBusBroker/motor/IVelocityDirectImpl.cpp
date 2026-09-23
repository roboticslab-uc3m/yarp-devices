// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CanBusBroker::setRefVelocity(int jnt, double vel)
{
    CHECK_JOINT(jnt);
    return deviceMapper.mapSingleJoint(&yarp::dev::IVelocityDirectRaw::setRefVelocityRaw, jnt, vel);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CanBusBroker::setRefVelocity(const std::vector<double> & vels)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IVelocityDirectRaw::setRefVelocityRaw, vels);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CanBusBroker::setRefVelocity(const std::vector<int> & jnts, const std::vector<double> & vels)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IVelocityDirectRaw::setRefVelocityRaw, jnts, vels);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CanBusBroker::getRefVelocity(int jnt, double & vel)
{
    CHECK_JOINT(jnt);
    return deviceMapper.mapSingleJoint(&yarp::dev::IVelocityDirectRaw::getRefVelocityRaw, jnt, vel);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CanBusBroker::getRefVelocity(std::vector<double> & vels)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IVelocityDirectRaw::getRefVelocityRaw, vels);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CanBusBroker::getRefVelocity(const std::vector<int> & jnts, std::vector<double> & vels)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IVelocityDirectRaw::getRefVelocityRaw, jnts, vels);
}

// -----------------------------------------------------------------------------
