// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

void TechnosoftIposExternal::interpretModesOfOperation(std::int8_t modesOfOperation)
{
    if (this->modesOfOperation == modesOfOperation)
    {
        return;
    }

    switch (modesOfOperation)
    {
    case -5:
        yCIInfo(IPOS, id()) << "iPOS specific: External Reference Torque Mode";
        actualControlMode.store(requestedcontrolMode);
        break;
    default:
        TechnosoftIposBase::interpretModesOfOperation(modesOfOperation);
        actualControlMode = VOCAB_CM_UNKNOWN;
        break;
    }

    this->modesOfOperation = modesOfOperation;
    controlModeObserverPtr->notify();
}

// -----------------------------------------------------------------------------

void TechnosoftIposExternal::onPositionLimitTriggered()
{
    auto mode = actualControlMode.load();

    // reset trajectory or command buffer on next SYNC

    switch (mode)
    {
    case VOCAB_CM_POSITION:
    case VOCAB_CM_VELOCITY:
    case VOCAB_CM_POSITION_DIRECT:
        if (mode == VOCAB_CM_POSITION || mode == VOCAB_CM_VELOCITY && !enableCsv)
        {
            trajectory.reset(internalUnitsToDegrees(lastEncoderRead->queryPosition()));
        }
        else if (mode == VOCAB_CM_POSITION_DIRECT)
        {
            commandBuffer.reset(internalUnitsToDegrees(lastEncoderRead->queryPosition()));
        }
        else // mode == VOCAB_CM_VELOCITY && enableCsv
        {
            commandBuffer.reset(0.0);
        }

        break;
    case VOCAB_CM_TORQUE:
    case VOCAB_CM_CURRENT:
        commandBuffer.reset(0.0);
        break;
    }
}

// -----------------------------------------------------------------------------

void TechnosoftIposExternal::reset()
{
    TechnosoftIposBase::reset();
    resetPidRaw(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, 0);
}

// -----------------------------------------------------------------------------
