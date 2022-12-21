// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#include <yarp/os/LogStream.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::synchronize()
{
    if (!enableSync)
    {
        return true;
    }

    switch (actualControlMode.load())
    {
    case VOCAB_CM_VELOCITY:
    {
        if (enableCsv)
        {
            double value = commandBuffer.interpolate() * syncPeriod;
            std::int32_t data = degreesToInternalUnits(value);
            return can->rpdo3()->write(data);
        }
        else
        {
            double value = degreesToInternalUnits(commandBuffer.interpolate(), 1);

            std::int16_t dataInt;
            std::uint16_t dataFrac;
            CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

            std::int32_t data = (dataInt << 16) + dataFrac;
            return can->rpdo3()->write(data);
        }
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
