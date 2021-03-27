// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

#include <yarp/os/Log.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TextilesHand::getControlMode(int j, int * mode)
{
    yTrace("%d", j);
    if (j != 0) return false;
    *mode = VOCAB_CM_POSITION_DIRECT;
    return true;
}

// -----------------------------------------------------------------------------

bool TextilesHand::getControlModes(int * modes)
{
    return getControlMode(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool TextilesHand::getControlModes(int n_joint, const int * joints, int * modes)
{
    return getControlMode(joints[0], &modes[0]);
}

// -----------------------------------------------------------------------------

bool TextilesHand::setControlMode(int j, int mode)
{
    yWarning("setControlMode() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool TextilesHand::setControlModes(int * modes)
{
    yWarning("setControlModes() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool TextilesHand::setControlModes(int n_joint, const int * joints, int * modes)
{
    yWarning("setControlModes() not supported");
    return false;
}

// -----------------------------------------------------------------------------
