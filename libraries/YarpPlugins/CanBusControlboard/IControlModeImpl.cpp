// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getControlMode(int j, int * mode)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlModeRaw::getControlModeRaw, j, mode);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getControlModes(int * modes)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IControlModeRaw::getControlModesRaw, modes);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getControlModes(int n_joint, const int * joints, int * modes)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IControlModeRaw::getControlModesRaw, n_joint, joints, modes);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setControlMode(int j, int mode)
{
#if YARP_VERSION_MINOR >= 5
    yCTrace(CBCB, "%d %s", j, yarp::os::Vocab32::decode(mode).c_str());
#else
    yCTrace(CBCB, "%d %s", j, yarp::os::Vocab::decode(mode).c_str());
#endif
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlModeRaw::setControlModeRaw, j, mode);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setControlModes(int * modes)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IControlModeRaw::setControlModesRaw, modes);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setControlModes(int n_joint, const int * joints, int * modes)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IControlModeRaw::setControlModesRaw, n_joint, joints, modes);
}

// -----------------------------------------------------------------------------
