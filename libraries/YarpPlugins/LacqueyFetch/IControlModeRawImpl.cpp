// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <yarp/conf/version.h>
#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool LacqueyFetch::getControlModeRaw(int j, int * mode)
{
#if YARP_VERSION_MINOR >= 6
    yCITrace(LCQ, id(), "%d", j);
#else
    yCTrace(LCQ, "%d", j);
#endif
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
#if YARP_VERSION_MINOR >= 6
    yCIWarning(LCQ, id(), "setControlModeRaw() not supported");
#else
    yCWarning(LCQ, "setControlModeRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::setControlModesRaw(int * modes)
{
#if YARP_VERSION_MINOR >= 6
    yCIWarning(LCQ, id(), "setControlModesRaw() not supported");
#else
    yCWarning(LCQ, "setControlModesRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::setControlModesRaw(int n_joint, const int * joints, int * modes)
{
#if YARP_VERSION_MINOR >= 6
    yCIWarning(LCQ, id(), "setControlModesRaw() not supported");
#else
    yCWarning(LCQ, "setControlModesRaw() not supported");
#endif
    return false;
}

// -----------------------------------------------------------------------------
