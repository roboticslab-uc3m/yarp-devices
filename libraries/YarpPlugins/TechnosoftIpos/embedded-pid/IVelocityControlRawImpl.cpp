// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#include <cmath> // std::abs

#include <algorithm> // std::clamp

#include <yarp/os/Log.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::velocityMoveRaw(int j, double sp)
{
    yCITrace(IPOS, id(), "%d %f", j, sp);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_VELOCITY);

    const double maxVel = this->maxVel;

    if (std::abs(sp) > maxVel)
    {
        yCIWarning(IPOS, id(), "Requested speed exceeds maximum velocity (%f)", maxVel);
        sp = std::clamp(sp, -maxVel, maxVel);
    }

    if (enableCsv)
    {
        commandBuffer.accept(sp);
        return true;
    }

    // reset halt bit
    if (can->driveStatus()->controlword()[8]
        && !can->driveStatus()->controlword(can->driveStatus()->controlword().reset(8)))
    {
        return false;
    }

    targetVelocity = sp;

    double value = degreesToInternalUnits(sp, 1);

    std::int16_t dataInt;
    std::uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    std::int32_t data = (dataInt << 16) + dataFrac;
    return can->sdo()->download<std::int32_t>("Target velocity", data, 0x60FF);
}

// ----------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRefVelocityRaw(int joint, double * vel)
{
    yCITrace(IPOS, id(), "%d", joint);
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_VELOCITY);

    if (enableCsv)
    {
        *vel = commandBuffer.getStoredCommand();
        return true;
    }

    // target velocity is stored in 0x606B; using local variable to avoid frequent SDO requests
    // (yarpmotorgui calls this quite fast)
    *vel = targetVelocity;
    return true;
}

// ------------------------------------------------------------------------------
