// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::getControlMode(int j, int * mode)
{
    yCTrace(CBB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlModeRaw::getControlModeRaw, j, mode);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getControlModes(int * modes)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IControlModeRaw::getControlModesRaw, modes);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getControlModes(int n_joint, const int * joints, int * modes)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IControlModeRaw::getControlModesRaw, n_joint, joints, modes);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setControlMode(int j, int mode)
{
    yCTrace(CBB, "%d %s", j, yarp::os::Vocab32::decode(mode).c_str());
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlModeRaw::setControlModeRaw, j, mode);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setControlModes(int * modes)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IControlModeRaw::setControlModesRaw, modes);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setControlModes(int n_joint, const int * joints, int * modes)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IControlModeRaw::setControlModesRaw, n_joint, joints, modes);
}

// -----------------------------------------------------------------------------
