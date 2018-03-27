// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboard.hpp"

// ################################ IInteractionModeRaw Related ################################


bool roboticslab::DextraControlboard::getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode)
{
    CD_INFO("(%d)\n",axis);

    *mode = interactionMode;
    return true;
}

// ----------------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// ----------------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// ----------------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode)
{
    CD_INFO("(%d), (%s)\n",axis, mode); //-- I don't know if this is correct (if I want to print mode?)

    interactionModeSemaphore.wait();
    interactionMode = mode;
    interactionModeSemaphore.post();

    return true;
}

// ----------------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;

}

// ----------------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;

}

// ----------------------------------------------------------------------------------------------
