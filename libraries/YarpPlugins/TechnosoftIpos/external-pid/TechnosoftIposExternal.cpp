// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposExternal.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

void TechnosoftIposExternal::interpretModesOfOperation(std::int8_t modesOfOperation)
{
    if (vars.modesOfOperation == modesOfOperation)
    {
        return;
    }

    switch (modesOfOperation)
    {
    default:
        TechnosoftIposBase::interpretModesOfOperation(modesOfOperation);
        vars.actualControlMode = VOCAB_CM_UNKNOWN;
        break;
    }

    vars.modesOfOperation = modesOfOperation;
    vars.controlModeObserverPtr->notify();
}

// -----------------------------------------------------------------------------

void TechnosoftIposExternal::reset()
{}

// -----------------------------------------------------------------------------
