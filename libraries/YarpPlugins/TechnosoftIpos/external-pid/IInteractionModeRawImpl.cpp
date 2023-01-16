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

bool TechnosoftIposExternal::setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode)
{
    yCITrace(IPOS, id(), "%d %s", axis, yarp::os::Vocab32::decode(mode).c_str());
    CHECK_JOINT(axis);

    if (mode == actualInteractionMode)
    {
        return true;
    }

    switch (mode)
    {
    case yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF:
    {
        std::lock_guard lock(pidMutex);
        activePid = &positionPid;
        break;
    }
    case yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT:
    {
        std::lock_guard lock(pidMutex);
        activePid = &impedancePid;
        break;
    }
    default:
        yCIError(IPOS, id()) << "Unsupported interaction mode" << yarp::os::Vocab32::decode(mode);
        return false;
    }

    actualInteractionMode = mode;
    return resetPidRaw(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, 0);
}

// -----------------------------------------------------------------------------
