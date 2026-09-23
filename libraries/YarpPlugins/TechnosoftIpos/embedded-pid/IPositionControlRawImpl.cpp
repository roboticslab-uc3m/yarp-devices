// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#include <cmath> // std::abs

#include <yarp/os/LogStream.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::positionMoveRaw(int j, double ref)
#else
bool TechnosoftIposEmbedded::positionMoveRaw(int j, double ref)
#endif
{
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    return !can->driveStatus()->controlword()[8] // check halt bit
        && can->sdo()->download<std::int32_t>("Target position", degreesToInternalUnits(ref), 0x607A)
        // new setpoint (assume absolute target position)
        && can->driveStatus()->controlword(can->driveStatus()->controlword().set(4).reset(6))
        && (targetPosition = ref, true)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ? yarp::dev::ReturnValue::return_code::return_value_ok : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        ;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::relativeMoveRaw(int j, double delta)
#else
bool TechnosoftIposEmbedded::relativeMoveRaw(int j, double delta)
#endif
{
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    return !can->driveStatus()->controlword()[8] // check halt bit
        && can->sdo()->download<std::int32_t>("Target position", degreesToInternalUnits(delta), 0x607A)
        // new setpoint (assume relative target position)
        && can->driveStatus()->controlword(can->driveStatus()->controlword().set(4).set(6))
        && (targetPosition = internalUnitsToDegrees(lastEncoderRead->queryPosition()) + delta, true)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ? yarp::dev::ReturnValue::return_code::return_value_ok : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        ;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::checkMotionDoneRaw(int j, bool & flag)
#else
bool TechnosoftIposEmbedded::checkMotionDoneRaw(int j, bool * flag)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    flag = can->driveStatus()->getCurrentState() != DriveState::OPERATION_ENABLED || can->driveStatus()->statusword()[10];
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    *flag = can->driveStatus()->getCurrentState() != DriveState::OPERATION_ENABLED || can->driveStatus()->statusword()[10];
    return true;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::setTrajSpeedRaw(int j, double sp)
#else
bool TechnosoftIposEmbedded::setRefSpeedRaw(int j, double sp)
#endif
{
    CHECK_JOINT(j);

    if (sp < 0.0)
    {
        yCIError(IPOS, id()) << "Illegal reference speed provided:" << sp;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }
    else if (sp == 0.0)
    {
        yCIWarning(IPOS, id()) << "Reference speed is zero, i.e. no motion will be performed";
    }
    else if (sp > maxVel)
    {
        yCIWarning(IPOS, id()) << "Reference speed exceeds maximum velocity, i.e." << maxVel.load() << "(will be clipped)";
        sp = maxVel;
    }

    double value = std::abs(degreesToInternalUnits(sp, 1));

    std::uint16_t dataInt;
    std::uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    std::uint32_t data = (dataInt << 16) + dataFrac;

    return can->sdo()->download("Profile velocity", data, 0x6081) && (refSpeed = sp, true)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        ;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::setTrajAccelerationRaw(int j, double acc)
#else
bool TechnosoftIposEmbedded::setRefAccelerationRaw(int j, double acc)
#endif
{
    CHECK_JOINT(j);

    if (acc <= 0.0)
    {
        yCIError(IPOS, id()) << "Illegal reference acceleration provided:" << acc;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    double value = std::abs(degreesToInternalUnits(acc, 2));

    std::uint16_t dataInt;
    std::uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    std::uint32_t data = (dataInt << 16) + dataFrac;

    return can->sdo()->download("Profile acceleration", data, 0x6083) && (refAcceleration = acc, true)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        ;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::getTrajSpeedRaw(int j, double * ref)
#else
bool TechnosoftIposEmbedded::getRefSpeedRaw(int j, double * ref)
#endif
{
    CHECK_JOINT(j);
    *ref = refSpeed;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::getTrajAccelerationRaw(int j, double * acc)
#else
bool TechnosoftIposEmbedded::getRefAccelerationRaw(int j, double * acc)
#endif
{
    CHECK_JOINT(j);
    *acc = refAcceleration;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::stopRaw(int j)
#else
bool TechnosoftIposEmbedded::stopRaw(int j)
#endif
{
    CHECK_JOINT(j);

    if (enableCsv && actualControlMode == VOCAB_CM_VELOCITY)
    {
        // don't mess with the halt bit here so that it doesn't need to be reset by `velocityMode()` later
        commandBuffer.reset(0.0);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
        return true;
#endif
    }

    return (actualControlMode == VOCAB_CM_POSITION || actualControlMode == VOCAB_CM_VELOCITY)
        && can->driveStatus()->controlword(can->driveStatus()->controlword().set(8)) // halt
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        ;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::getTargetPositionRaw(int joint, double * ref)
#else
bool TechnosoftIposEmbedded::getTargetPositionRaw(int joint, double * ref)
#endif
{
    CHECK_JOINT(joint);

    // target position is stored in 0x607A; using local variable to avoid frequent SDO requests
    // (yarpmotorgui calls this quite fast)
    *ref = targetPosition;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// --------------------------------------------------------------------------------
