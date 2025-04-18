// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::initialize()
{
    return TechnosoftIposBase::initialize()
        && setInteractionModeRaw(0, static_cast<yarp::dev::InteractionModeEnum>(initialInteractionMode));
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::synchronize(double timestamp)
{
    int mode = actualControlMode;
    double current;

    if (mode == VOCAB_CM_POSITION || mode == VOCAB_CM_VELOCITY || mode == VOCAB_CM_POSITION_DIRECT)
    {
        double target, forceCommand;

        if (mode == VOCAB_CM_POSITION || mode == VOCAB_CM_VELOCITY && !enableCsv)
        {
            target = trajectory.update(timestamp).position;
        }
        else if (mode == VOCAB_CM_VELOCITY && enableCsv)
        {
            double prevTarget;
            getPidReferenceRaw(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &prevTarget);
            target = prevTarget + commandBuffer.interpolate() * params.m_syncPeriod;
        }
        else // mode == VOCAB_CM_POSITION_DIRECT
        {
            target = commandBuffer.interpolate();
        }

        setPidReferenceRaw(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, target);
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
