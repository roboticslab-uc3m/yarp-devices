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
    *mode = vars.actualControlMode;
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getControlModesRaw(int * modes)
{
    return getControlModeRaw(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getControlModesRaw(int n_joint, const int * joints, int * modes)
{
    return getControlModeRaw(joints[0], &modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setControlModeRaw(int j, int mode)
{
    yCITrace(IPOS, id(), "%d %s", j, yarp::os::Vocab32::decode(mode).c_str());
    CHECK_JOINT(j);

    vars.requestedcontrolMode = mode;
    auto currentPositionRead = vars.lastEncoderRead->queryPosition();

    switch (mode)
    {
    case VOCAB_CM_POSITION:
        vars.actualControlMode = VOCAB_CM_POSITION;
        positionTrajectory.reset(vars.internalUnitsToDegrees(currentPositionRead));
        resetPidRaw(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, 0);
        break;
    case VOCAB_CM_POSITION_DIRECT:
        vars.actualControlMode = VOCAB_CM_POSITION_DIRECT;
        break;
    default:
        return false; // TODO
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setControlModesRaw(int * modes)
{
    return setControlModeRaw(0, modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setControlModesRaw(int n_joint, const int * joints, int * modes)
{
    return setControlModeRaw(joints[0], modes[0]);
}

// -----------------------------------------------------------------------------
