// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <yarp/os/Log.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool LacqueyFetch::getControlModeRaw(int j, int * mode)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    *mode = VOCAB_CM_PWM;
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::getControlModesRaw(int * modes)
{
    return getControlModeRaw(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::getControlModesRaw(int n_joint, const int * joints, int * modes)
{
    return getControlModeRaw(joints[0], &modes[0]);
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::setControlModeRaw(int j, int mode)
{
    CHECK_JOINT(j);
    yWarning("setControlModeRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::setControlModesRaw(int * modes)
{
    yWarning("setControlModesRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::setControlModesRaw(int n_joint, const int * joints, int * modes)
{
    yWarning("setControlModesRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------
