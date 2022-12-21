// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposExternal.hpp"

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
    default:
        TechnosoftIposBase::interpretModesOfOperation(modesOfOperation);
        actualControlMode = VOCAB_CM_UNKNOWN;
        break;
    }

    this->modesOfOperation = modesOfOperation;
    controlModeObserverPtr->notify();
}

// -----------------------------------------------------------------------------

void TechnosoftIposExternal::reset()
{
    TechnosoftIposBase::reset();
}

// -----------------------------------------------------------------------------
