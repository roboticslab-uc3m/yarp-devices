// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::synchronize(double timestamp)
{
    if (!enableSync)
    {
        return true;
    }

    switch (actualControlMode.load())
    {
    case VOCAB_CM_VELOCITY:
    {
        // enableCsv = true
        double value = commandBuffer.interpolate() * syncPeriod;
        std::int32_t data = degreesToInternalUnits(value);
        return can->rpdo3()->write(data);
    }
    case VOCAB_CM_TORQUE:
    {
        double curr = torqueToCurrent(commandBuffer.interpolate());
        std::int32_t data = currentToInternalUnits(curr) << 16;
        return can->rpdo3()->write(data);
    }
    case VOCAB_CM_CURRENT:
    {
        std::int32_t data = currentToInternalUnits(commandBuffer.interpolate()) << 16;
        return can->rpdo3()->write(data);
    }
    case VOCAB_CM_POSITION_DIRECT:
    {
        std::int32_t data = degreesToInternalUnits(commandBuffer.interpolate());
        return can->rpdo3()->write(data);
    }
    default:
        return true;
    }
}

// -----------------------------------------------------------------------------
