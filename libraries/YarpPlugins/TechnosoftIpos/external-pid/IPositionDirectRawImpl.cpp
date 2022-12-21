// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setPositionRaw(int j, double ref)
{
    yCITrace(IPOS, id(), "%d %f", j, ref);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION_DIRECT);
    commandBuffer.accept(ref); // TODO: clip if exceeds max speed
    return true;
}
// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefPositionRaw(int joint, double * ref)
{
    yCITrace(IPOS, id(), "%d", joint);
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_POSITION_DIRECT);
    return commandBuffer.getStoredCommand();
}

// -----------------------------------------------------------------------------
