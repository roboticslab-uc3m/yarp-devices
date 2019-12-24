// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TextilesHand::getControlMode(int j, int * mode)
{
    CD_DEBUG("(%d)\n", j);
    if (j != 0) return false;
    *mode = VOCAB_CM_POSITION_DIRECT;
    return true;
}

// -----------------------------------------------------------------------------

bool TextilesHand::getControlModes(int * modes)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    return getControlMode(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool TextilesHand::getControlModes(int n_joint, const int * joints, int * modes)
{
    CD_DEBUG("(%d)\n", n_joint);
    return getControlMode(joints[0], &modes[0]);
}

// -----------------------------------------------------------------------------

bool TextilesHand::setControlMode(int j, int mode)
{
    CD_WARNING("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool TextilesHand::setControlModes(int * modes)
{
    CD_WARNING("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool TextilesHand::setControlModes(int n_joint, const int * joints, int * modes)
{
    CD_WARNING("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------
