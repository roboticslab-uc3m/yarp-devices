// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#include <cmath>

#include <yarp/os/LogStream.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::positionMoveRaw(int j, double ref)
{
    yCITrace(IPOS, id(), "%d %f", j, ref);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    return !can->driveStatus()->controlword()[8] // check halt bit
        && can->sdo()->download<std::int32_t>("Target position", degreesToInternalUnits(ref), 0x607A)
        // new setpoint (assume absolute target position)
        && can->driveStatus()->controlword(can->driveStatus()->controlword().set(4).reset(6));
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::positionMoveRaw(const double * refs)
{
    return positionMoveRaw(0, refs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::positionMoveRaw(int n_joint, const int * joints, const double * refs)
{
    return positionMoveRaw(joints[0], refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::relativeMoveRaw(int j, double delta)
{
    yCITrace(IPOS, id(), "%d %f", j, delta);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    return !can->driveStatus()->controlword()[8] // check halt bit
        && can->sdo()->download<std::int32_t>("Target position", degreesToInternalUnits(delta), 0x607A)
        // new setpoint (assume relative target position)
        && can->driveStatus()->controlword(can->driveStatus()->controlword().set(4).set(6));
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::relativeMoveRaw(const double * deltas)
{
    return relativeMoveRaw(0, deltas[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::relativeMoveRaw(int n_joint, const int * joints, const double * deltas)
{
    return relativeMoveRaw(joints[0], deltas[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::checkMotionDoneRaw(int j, bool * flag)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    *flag = can->driveStatus()->getCurrentState() != DriveState::OPERATION_ENABLED || can->driveStatus()->statusword()[10];
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::checkMotionDoneRaw(bool * flag)
{
    return checkMotionDoneRaw(0, flag);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::checkMotionDoneRaw(int n_joint, const int * joints, bool * flag)
{
    return checkMotionDoneRaw(joints[0], flag);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::setRefSpeedRaw(int j, double sp)
{
    yCITrace(IPOS, id(), "%d %f", j, sp);
    CHECK_JOINT(j);

    if (sp < 0.0)
    {
        yCIWarning(IPOS, id()) << "Illegal negative speed provided:" << sp;
        return false;
    }
    else if (sp > maxVel)
    {
        yCIWarning(IPOS, id()) << "Reference speed exceeds maximum velocity, i.e." << maxVel.load();
        return false;
    }

    double value = std::abs(degreesToInternalUnits(sp, 1));

    std::uint16_t dataInt;
    std::uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    std::uint32_t data = (dataInt << 16) + dataFrac;

    if (!can->sdo()->download("Profile velocity", data, 0x6081))
    {
        return false;
    }

    refSpeed = sp;
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::setRefSpeedsRaw(const double * spds)
{
    return setRefSpeedRaw(0, spds[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::setRefSpeedsRaw(int n_joint, const int * joints, const double * spds)
{
    return setRefSpeedRaw(joints[0], spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::setRefAccelerationRaw(int j, double acc)
{
    yCITrace(IPOS, id(), "%d %f", j, acc);
    CHECK_JOINT(j);

    if (acc < 0.0)
    {
        yCIWarning(IPOS, id()) << "Illegal negative acceleration provided:" << acc;
        return false;
    }

    double value = std::abs(degreesToInternalUnits(acc, 2));

    std::uint16_t dataInt;
    std::uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    std::uint32_t data = (dataInt << 16) + dataFrac;

    if (!can->sdo()->download("Profile acceleration", data, 0x6083))
    {
        return false;
    }

    refAcceleration = acc;
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::setRefAccelerationsRaw(const double * accs)
{
    return setRefAccelerationRaw(0, accs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs)
{
    return setRefAccelerationRaw(joints[0], accs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRefSpeedRaw(int j, double * ref)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    if (actualControlMode == VOCAB_CM_NOT_CONFIGURED)
    {
        *ref = refSpeed;
        return true;
    }

    return can->sdo()->upload<std::uint32_t>("Profile velocity", [this, ref](auto data)
        {
            std::uint16_t dataInt = data >> 16;
            std::uint16_t dataFrac = data & 0xFFFF;
            double value = CanUtils::decodeFixedPoint(dataInt, dataFrac);
            *ref = std::abs(internalUnitsToDegrees(value, 1));
        },
        0x6081);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRefSpeedsRaw(double * spds)
{
    return getRefSpeedRaw(0, &spds[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRefSpeedsRaw(int n_joint, const int * joints, double * spds)
{
    return getRefSpeedRaw(joints[0], &spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRefAccelerationRaw(int j, double * acc)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    if (actualControlMode == VOCAB_CM_NOT_CONFIGURED)
    {
        *acc = refAcceleration;
        return true;
    }

    return can->sdo()->upload<std::uint32_t>("Profile acceleration", [this, acc](auto data)
        {
            std::uint16_t dataInt = data >> 16;
            std::uint16_t dataFrac = data & 0xFFFF;
            double value = CanUtils::decodeFixedPoint(dataInt, dataFrac);
            *acc = std::abs(internalUnitsToDegrees(value, 2));
        },
        0x6083);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRefAccelerationsRaw(double * accs)
{
    return getRefAccelerationRaw(0, &accs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRefAccelerationsRaw(int n_joint, const int * joints, double * accs)
{
    return getRefAccelerationRaw(joints[0], &accs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::stopRaw(int j)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    return (actualControlMode == VOCAB_CM_POSITION || actualControlMode == VOCAB_CM_VELOCITY)
        && can->driveStatus()->controlword(can->driveStatus()->controlword().set(8)) // stop with profile acceleration
        && (commandBuffer.reset(0.0), true);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::stopRaw()
{
    return stopRaw(0);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::stopRaw(int n_joint, const int * joints)
{
    return stopRaw(joints[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getTargetPositionRaw(int joint, double * ref)
{
    yCITrace(IPOS, id(), "%d", joint);
    CHECK_JOINT(joint);

    return can->sdo()->upload<std::int32_t>("Target position", [this, ref](auto data)
        { *ref = internalUnitsToDegrees(data); },
        0x607A);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getTargetPositionsRaw(double * refs)
{
    return getTargetPositionRaw(0, &refs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getTargetPositionsRaw(int n_joint, const int * joints, double * refs)
{
    return getTargetPositionRaw(joints[0], &refs[0]);
}

// --------------------------------------------------------------------------------
