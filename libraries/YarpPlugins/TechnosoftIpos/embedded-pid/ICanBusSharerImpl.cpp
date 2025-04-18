// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::synchronize(double timestamp)
{
    if (!enableSync)
    {
        return true;
    }

    std::int32_t data;

    switch (actualControlMode.load())
    {
    case VOCAB_CM_VELOCITY: // enableCsv = true
        data = degreesToInternalUnits(commandBuffer.interpolate() * params.m_syncPeriod);
        break;
    case VOCAB_CM_POSITION_DIRECT:
        data = degreesToInternalUnits(commandBuffer.interpolate());
        break;
    case VOCAB_CM_TORQUE:
        data = currentToInternalUnits(torqueToCurrent(commandBuffer.interpolate())) << 16;
        break;
    case VOCAB_CM_CURRENT:
        data = currentToInternalUnits(commandBuffer.interpolate()) << 16;
        break;
    default:
        return true;
    }

    return can->rpdo3()->write(data);
}

// -----------------------------------------------------------------------------
