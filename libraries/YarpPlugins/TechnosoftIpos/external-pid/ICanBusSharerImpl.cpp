// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/SystemClock.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::initialize()
{
    return TechnosoftIposBase::initialize()
        && setInteractionModeRaw(0, static_cast<yarp::dev::InteractionModeEnum>(initialInteractionMode));
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::synchronize()
{
    int mode = actualControlMode;
    double current;

    if (mode == VOCAB_CM_POSITION || mode == VOCAB_CM_VELOCITY)
    {
        double forceCommand;
        auto reference = trapTrajectory.update(yarp::os::SystemClock::nowSystem());
        setPidReferenceRaw(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, reference.position);
        getPidOutputRaw(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &forceCommand);
        current = torqueToCurrent(forceCommand);
    }
    else if (mode == VOCAB_CM_POSITION_DIRECT)
    {
        double forceCommand;
        setPidReferenceRaw(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, commandBuffer.interpolate());
        getPidOutputRaw(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &forceCommand);
        current = torqueToCurrent(forceCommand);
    }
    else if (mode == VOCAB_CM_TORQUE)
    {
        current = torqueToCurrent(commandBuffer.interpolate());
    }
    else if (mode == VOCAB_CM_CURRENT)
    {
        current = commandBuffer.interpolate();
    }
    else
    {
        return true;
    }

    std::int32_t data = currentToInternalUnits(current) << 16;
    return can->rpdo3()->write(data);
}

// -----------------------------------------------------------------------------
