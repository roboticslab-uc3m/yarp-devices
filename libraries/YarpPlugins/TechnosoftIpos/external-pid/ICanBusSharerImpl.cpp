// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::synchronize()
{
    switch (vars.actualControlMode.load())
    {
    case VOCAB_CM_POSITION:
    case VOCAB_CM_VELOCITY:
        trapTrajectory.update();
        setPidReferenceRaw(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, trapTrajectory.queryPosition());
        // fall-through
    case VOCAB_CM_POSITION_DIRECT:
    {
        double forceCommand;
        getPidOutputRaw(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &forceCommand); // TODO: check return value
        double curr = vars.torqueToCurrent(vars.synchronousCommandTarget);
        std::int32_t data = vars.currentToInternalUnits(curr) << 16;
        return can->rpdo3()->write(data);
    }
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
