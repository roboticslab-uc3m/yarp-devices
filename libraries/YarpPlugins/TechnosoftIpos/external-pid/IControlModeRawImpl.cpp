// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getControlModeRaw(int j, int * mode)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    *mode = actualControlMode;
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setControlModeRaw(int j, int mode)
{
    yCITrace(IPOS, id(), "%d %s", j, yarp::os::Vocab32::decode(mode).c_str());
    CHECK_JOINT(j);

    requestedcontrolMode = actualControlMode = mode;
    auto currentPositionRead = lastEncoderRead->queryPosition();

    switch (mode)
    {
    case VOCAB_CM_POSITION:
    case VOCAB_CM_VELOCITY:
        trapTrajectory.reset(internalUnitsToDegrees(currentPositionRead));
        resetPidRaw(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, 0);
        break;
    case VOCAB_CM_POSITION_DIRECT:
        commandBuffer.reset(internalUnitsToDegrees(currentPositionRead));
        break;
    default:
        return false; // TODO
    }

    return true;
}

// -----------------------------------------------------------------------------
