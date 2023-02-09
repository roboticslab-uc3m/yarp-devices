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

    const auto state = limitSwitchState.load();

    if (state == INACTIVE || state == POSITIVE && ref <= max || state == NEGATIVE && ref >= min)
    {
        // reset halt bit (8) and re-enable ext. ref. torque (4)
        if (can->driveStatus()->controlword()[8] &&
            (!can->driveStatus()->controlword(can->driveStatus()->controlword().reset(8)) ||
            // must be issued separately
             !can->driveStatus()->controlword(can->driveStatus()->controlword().set(4))))
        {
            return false;
        }

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
    yCITrace(IPOS, id(), "%d", joint);
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_POSITION_DIRECT);
    *ref = commandBuffer.getStoredCommand();
    return true;
}

// -----------------------------------------------------------------------------
