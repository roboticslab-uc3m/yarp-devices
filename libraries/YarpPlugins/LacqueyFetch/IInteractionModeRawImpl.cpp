// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <yarp/os/Vocab.h>

// ------------------ IInteractionModeRaw Related ----------------------------------------

bool roboticslab::LacqueyFetch::getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode)
{
    CD_INFO("(%d)\n",axis);

    interactionModeSemaphore.wait();
    *mode = interactionMode;
    interactionModeSemaphore.post();

    return true;
}

bool roboticslab::LacqueyFetch::getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

bool roboticslab::LacqueyFetch::getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

bool roboticslab::LacqueyFetch::setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode)
{    
    CD_INFO("(%d), (%s)\n", axis, yarp::os::Vocab::decode(mode).c_str());

    interactionModeSemaphore.wait();
    interactionMode = mode;
    interactionModeSemaphore.post();

    return true;
}

bool roboticslab::LacqueyFetch::setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;

}

bool roboticslab::LacqueyFetch::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;

}
