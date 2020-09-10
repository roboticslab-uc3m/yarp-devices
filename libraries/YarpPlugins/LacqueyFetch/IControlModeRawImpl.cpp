// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool LacqueyFetch::getControlModeRaw(int j, int * mode)
{
    //CD_DEBUG("(%d)\n", j); // too verbose in controlboardwrapper2 stream
    CHECK_JOINT(j);
    *mode = VOCAB_CM_PWM;
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::getControlModesRaw(int * modes)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    return getControlModeRaw(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::getControlModesRaw(int n_joint, const int * joints, int * modes)
{
    CD_DEBUG("\n");
    return getControlModeRaw(joints[0], &modes[0]);
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::setControlModeRaw(int j, int mode)
{
    CHECK_JOINT(j);
    CD_WARNING("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::setControlModesRaw(int * modes)
{
    CD_WARNING("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::setControlModesRaw(int n_joint, const int * joints, int * modes)
{
    CD_WARNING("Not supported.\n");
    return false;
}

// -----------------------------------------------------------------------------
