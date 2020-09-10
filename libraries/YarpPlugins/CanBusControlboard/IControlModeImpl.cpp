// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Vocab.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getControlMode(int j, int * mode)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlModeRaw::getControlModeRaw, j, mode);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getControlModes(int * modes)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    return deviceMapper.mapAllJoints(&yarp::dev::IControlModeRaw::getControlModesRaw, modes);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getControlModes(int n_joint, const int * joints, int * modes)
{
    CD_DEBUG("\n");
    return deviceMapper.mapJointGroup(&yarp::dev::IControlModeRaw::getControlModesRaw, n_joint, joints, modes);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setControlMode(int j, int mode)
{
    CD_DEBUG("(%d, %s)\n", j, yarp::os::Vocab::decode(mode).c_str());
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlModeRaw::setControlModeRaw, j, mode);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setControlModes(int * modes)
{
    CD_DEBUG("\n");
    return deviceMapper.mapAllJoints(&yarp::dev::IControlModeRaw::setControlModesRaw, modes);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setControlModes(int n_joint, const int * joints, int * modes)
{
    CD_DEBUG("(%d)\n",n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IControlModeRaw::setControlModesRaw, n_joint, joints, modes);
}

// -----------------------------------------------------------------------------
