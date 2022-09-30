// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::synchronize()
{
    double forceCommand;

    switch (vars.actualControlMode.load())
    {
    case VOCAB_CM_POSITION:
    case VOCAB_CM_VELOCITY:
    {
        trapTrajectory.update();
        setPidReferenceRaw(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, trapTrajectory.queryPosition());
        getPidOutputRaw(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &forceCommand); // TODO: check return value
        double curr = vars.torqueToCurrent(forceCommand);
        std::int32_t data = vars.currentToInternalUnits(curr) << 16;
        return can->rpdo3()->write(data);
    }
    case VOCAB_CM_POSITION_DIRECT:
    {
        setPidReferenceRaw(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, commandBuffer.interpolate());
        getPidOutputRaw(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &forceCommand); // TODO: check return value
        double curr = vars.torqueToCurrent(forceCommand);
        std::int32_t data = vars.currentToInternalUnits(curr) << 16;
        return can->rpdo3()->write(data);
    }
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
