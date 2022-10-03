// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum * mode)
{
    yCITrace(IPOS, id(), "%d", axis);
    CHECK_JOINT(axis);
    *mode = actualInteractionMode;
    return true;
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

    std::unique_lock lock(pidMutex);

    switch (mode)
    {
    case yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF:
        activePid = &positionPid;
        break;
    case yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT:
        activePid = &impedancePid;
        break;
    default:
        yCIError(IPOS, id()) << "Unsupported interaction mode" << yarp::os::Vocab32::decode(mode);
        return false;
    }

    lock.unlock();
    actualInteractionMode = mode;
    return resetPidRaw(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, 0);
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
