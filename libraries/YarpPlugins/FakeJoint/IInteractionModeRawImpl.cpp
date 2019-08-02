// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

#include <yarp/os/Vocab.h>

// ################################ IInteractionModeRaw Related ################################

bool roboticslab::FakeJoint::getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode)
{
    CD_INFO("(%d)\n",axis);

    *mode = interactionMode;
    return true;
}

// ----------------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// ----------------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// ----------------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode)
{
    CD_INFO("(%d), (%s)\n", axis, yarp::os::Vocab::decode(mode).c_str());

    interactionModeSemaphore.wait();
    interactionMode = mode;
    interactionModeSemaphore.post();

    return true;
}

// ----------------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;

}

// ----------------------------------------------------------------------------------------------

bool roboticslab::FakeJoint::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;

}

// ----------------------------------------------------------------------------------------------
