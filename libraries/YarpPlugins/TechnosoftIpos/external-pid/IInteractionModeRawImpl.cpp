// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum & mode)
#else
bool TechnosoftIposExternal::getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum * mode)
#endif
{
    CHECK_JOINT(axis);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    mode = actualInteractionMode;
    return yarp::dev::ReturnValue_ok;
#else
    *mode = actualInteractionMode;
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode)
#else
bool TechnosoftIposExternal::setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode)
#endif
{
    CHECK_JOINT(axis);

    if (mode == actualInteractionMode)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_ok;
#else
        return true;
#endif
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
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    actualInteractionMode = mode;
    return resetPidRaw(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, 0);
}

// -----------------------------------------------------------------------------
