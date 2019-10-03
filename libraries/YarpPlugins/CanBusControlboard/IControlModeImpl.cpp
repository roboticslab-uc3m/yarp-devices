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

    if (!deviceMapper.mapSingleJoint(&yarp::dev::IControlModeRaw::setControlModeRaw, j, mode))
    {
        return false;
    }

    posdThread->updateControlModeRegister(j, mode == VOCAB_CM_POSITION_DIRECT);
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setControlModes(int * modes)
{
    CD_DEBUG("\n");

    if (!deviceMapper.mapAllJoints(&yarp::dev::IControlModeRaw::setControlModesRaw, modes))
    {
        return false;
    }

    for (unsigned int i = 0; i < nodes.size(); i++)
    {
        posdThread->updateControlModeRegister(i, modes[i] == VOCAB_CM_POSITION_DIRECT);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setControlModes(int n_joint, const int * joints, int * modes)
{
    CD_DEBUG("(%d)\n",n_joint);

    if (!deviceMapper.mapJointGroup(&yarp::dev::IControlModeRaw::setControlModesRaw, n_joint, joints, modes))

    for (int i = 0; i < n_joint; i++)
    {
        posdThread->updateControlModeRegister(joints[i], modes[i] == VOCAB_CM_POSITION_DIRECT);
    }

    return true;
}

// -----------------------------------------------------------------------------
