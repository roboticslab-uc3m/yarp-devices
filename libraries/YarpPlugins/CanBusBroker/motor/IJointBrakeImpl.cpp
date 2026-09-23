// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CanBusBroker::isJointBraked(int j, bool & braked) const
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IJointBrakeRaw::isJointBrakedRaw, j, braked);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CanBusBroker::setManualBrakeActive(int j, bool active)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IJointBrakeRaw::setManualBrakeActiveRaw, j, active);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CanBusBroker::setAutoBrakeEnabled(int j, bool enabled)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IJointBrakeRaw::setAutoBrakeEnabledRaw, j, enabled);
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue CanBusBroker::getAutoBrakeEnabled(int j, bool & enabled) const
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IJointBrakeRaw::getAutoBrakeEnabledRaw, j, enabled);
}

// -----------------------------------------------------------------------------
