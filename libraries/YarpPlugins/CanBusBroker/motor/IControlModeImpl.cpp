// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::getControlMode(int j, int * mode)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlModeRaw::getControlModeRaw, j, mode);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getControlModes(int * modes)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IControlModeRaw::getControlModesRaw, modes);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getControlModes(int n_joint, const int * joints, int * modes)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IControlModeRaw::getControlModesRaw, n_joint, joints, modes);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setControlMode(int j, int mode)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlModeRaw::setControlModeRaw, j, mode);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setControlModes(int * modes)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IControlModeRaw::setControlModesRaw, modes);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setControlModes(int n_joint, const int * joints, int * modes)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IControlModeRaw::setControlModesRaw, n_joint, joints, modes);
}

// -----------------------------------------------------------------------------
