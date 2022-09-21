// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::synchronize()
{
    switch (vars.actualControlMode.load())
    {
    case VOCAB_CM_POSITION:
        positionTrajectory.update();
        vars.synchronousCommandTarget = positionTrajectory.queryPosition();
        // fall-through
    case VOCAB_CM_POSITION_DIRECT:
        // TODO: PID output
        break;
    }

    return true;
}

// -----------------------------------------------------------------------------
