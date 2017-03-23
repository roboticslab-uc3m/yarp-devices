// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IInteractionModeRaw Related ----------------------------------------


bool teo::TechnosoftIpos::getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode)
{
    CD_INFO("(%d)\n",axis);

    *mode = interactionMode;
    return true;
}


bool teo::TechnosoftIpos::getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

bool teo::TechnosoftIpos::getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

bool teo::TechnosoftIpos::setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode)
{
    CD_INFO("(%d), (%s)\n",axis, mode); //-- I don't know if this is correct (if I want to print mode?)

    interactionModeSemaphore.wait();
    interactionMode = mode;
    interactionModeSemaphore.post();

    return true;
}

bool teo::TechnosoftIpos::setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;

}

bool teo::TechnosoftIpos::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;

}


