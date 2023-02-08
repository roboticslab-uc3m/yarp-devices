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
    ;
}

// -----------------------------------------------------------------------------

void TechnosoftIposExternal::reset()
{
    TechnosoftIposBase::reset();
    resetPidRaw(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, 0);
}

// -----------------------------------------------------------------------------
