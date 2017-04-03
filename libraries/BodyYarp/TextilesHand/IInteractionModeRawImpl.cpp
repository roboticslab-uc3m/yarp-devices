// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// ################################ IInteractionModeRaw Related ################################


bool teo::TextilesHand::getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode)
{
    CD_INFO("(%d)\n",axis);

    *mode = interactionMode;
    return true;
}

// ----------------------------------------------------------------------------------------------

bool teo::TextilesHand::getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// ----------------------------------------------------------------------------------------------

bool teo::TextilesHand::getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;
}

// ----------------------------------------------------------------------------------------------

bool teo::TextilesHand::setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode)
{
    CD_INFO("(%d), (%s)\n",axis, mode); //-- I don't know if this is correct (if I want to print mode?)

    interactionModeSemaphore.wait();
    interactionMode = mode;
    interactionModeSemaphore.post();

    return true;
}

// ----------------------------------------------------------------------------------------------

bool teo::TextilesHand::setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;

}

// ----------------------------------------------------------------------------------------------

bool teo::TextilesHand::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    CD_WARNING("Missing implementation\n");

    return true;

}

// ----------------------------------------------------------------------------------------------
