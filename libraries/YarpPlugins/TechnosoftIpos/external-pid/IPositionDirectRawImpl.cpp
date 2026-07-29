// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setPositionRaw(int j, double ref)
{
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION_DIRECT);

    const auto state = limitSwitchState.load();

    if (state == INACTIVE || state == POSITIVE && ref <= max || state == NEGATIVE && ref >= min)
    {
        commandBuffer.accept(ref); // TODO: clip if exceeds max speed
        return true;
    }
    else
    {
        return false;
    }
}
// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefPositionRaw(int joint, double * ref)
{
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_POSITION_DIRECT);
    *ref = commandBuffer.getStoredCommand();
    return true;
}

// -----------------------------------------------------------------------------
