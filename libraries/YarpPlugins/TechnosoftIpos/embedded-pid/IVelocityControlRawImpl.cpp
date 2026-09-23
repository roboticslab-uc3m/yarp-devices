// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#include <cmath> // std::abs

#include <algorithm> // std::clamp

#include <yarp/os/Log.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::velocityMoveRaw(int j, double sp)
#else
bool TechnosoftIposEmbedded::velocityMoveRaw(int j, double sp)
#endif
{
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
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
        return true;
#endif
    }

    // reset halt bit
    if (can->driveStatus()->controlword()[8]
        && !can->driveStatus()->controlword(can->driveStatus()->controlword().reset(8)))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
    }

    targetVelocity = sp;

    double value = degreesToInternalUnits(sp, 1);

    std::int16_t dataInt;
    std::uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    std::int32_t data = (dataInt << 16) + dataFrac;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return can->sdo()->download<std::int32_t>("Target velocity", data, 0x60FF)
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return can->sdo()->download<std::int32_t>("Target velocity", data, 0x60FF);
#endif
}

// ----------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::getTargetVelocityRaw(int joint, double * vel)
#else
bool TechnosoftIposEmbedded::getRefVelocityRaw(int joint, double * vel)
#endif
{
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_VELOCITY);

    if (enableCsv)
    {
        *vel = commandBuffer.getStoredCommand();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
        return true;
#endif
    }

    // target velocity is stored in 0x606B; using local variable to avoid frequent SDO requests
    // (yarpmotorgui calls this quite fast)
    *vel = targetVelocity;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// ------------------------------------------------------------------------------
