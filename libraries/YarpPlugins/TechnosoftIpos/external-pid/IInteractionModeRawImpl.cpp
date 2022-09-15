// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum * mode)
{
    yCITrace(IPOS, id(), "%d", axis);
    CHECK_JOINT(axis);
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes)
{
    return getInteractionModeRaw(joints[0], &modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getInteractionModesRaw(yarp::dev::InteractionModeEnum * modes)
{
    return getInteractionModeRaw(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode)
{
    yCITrace(IPOS, id(), "%d %s", axis, yarp::os::Vocab32::decode(mode).c_str());
    CHECK_JOINT(axis);
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setInteractionModesRaw(int n_joints, int * joints, yarp::dev::InteractionModeEnum * modes)
{
    return setInteractionModeRaw(joints[0], modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setInteractionModesRaw(yarp::dev::InteractionModeEnum * modes)
{
    return setInteractionModeRaw(0, modes[0]);
}

// -----------------------------------------------------------------------------
