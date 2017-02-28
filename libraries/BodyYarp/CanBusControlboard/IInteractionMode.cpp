// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"


// ---------------------------- IInteractionMode Related ----------------------------------

bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode)
{
    return true;
}

bool teo::CanBusControlboard::getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    return true;
}

bool teo::CanBusControlboard::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    return true;
}

bool teo::CanBusControlboard::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    return true;
}

bool teo::CanBusControlboard::setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    return true;
}

bool teo::CanBusControlboard::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    return true;
}
