// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------ IInteractionMode related -----------------------------------------

bool roboticslab::AmorControlboard::getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode)
{
    CD_DEBUG("(%d)\n", axis);

    if (!indexWithinRange(axis))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_DEBUG("(%d)\n", n_joints);

    if (!batchWithinRange(n_joints))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    CD_DEBUG("(%d)\n", axis);

    if (!indexWithinRange(axis))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    CD_DEBUG("(%d)\n", n_joints);

    if (!batchWithinRange(n_joints))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------
