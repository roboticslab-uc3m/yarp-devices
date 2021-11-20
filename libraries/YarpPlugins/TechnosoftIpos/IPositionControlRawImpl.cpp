// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cmath>

#include <yarp/conf/version.h>
#include <yarp/os/LogStream.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// --------------------------------------------------------------------------------

bool TechnosoftIpos::positionMoveRaw(int j, double ref)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d %f", j, ref);
#else
    yCTrace(IPOS, "%d %f", j, ref);
#endif
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    return !can->driveStatus()->controlword()[8] // check halt bit
        && can->sdo()->download<std::int32_t>("Target position", vars.degreesToInternalUnits(ref), 0x607A)
        // new setpoint (assume absolute target position)
        && can->driveStatus()->controlword(can->driveStatus()->controlword().set(4).reset(6));
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::positionMoveRaw(const double * refs)
{
    return positionMoveRaw(0, refs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::positionMoveRaw(int n_joint, const int * joints, const double * refs)
{
    return positionMoveRaw(joints[0], refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::relativeMoveRaw(int j, double delta)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d %f", j, delta);
#else
    yCTrace(IPOS, "%d %f", j, delta);
#endif
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    return !can->driveStatus()->controlword()[8] // check halt bit
        && can->sdo()->download<std::int32_t>("Target position", vars.degreesToInternalUnits(delta), 0x607A)
        // new setpoint (assume relative target position)
        && can->driveStatus()->controlword(can->driveStatus()->controlword().set(4).set(6));
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::relativeMoveRaw(const double * deltas)
{
    return relativeMoveRaw(0, deltas[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::relativeMoveRaw(int n_joint, const int * joints, const double * deltas)
{
    return relativeMoveRaw(joints[0], deltas[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::checkMotionDoneRaw(int j, bool * flag)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d", j);
#else
    yCTrace(IPOS, "%d", j);
#endif
    CHECK_JOINT(j);
    *flag = can->driveStatus()->getCurrentState() != DriveState::OPERATION_ENABLED || can->driveStatus()->statusword()[10];
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::checkMotionDoneRaw(bool * flag)
{
    return checkMotionDoneRaw(0, flag);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::checkMotionDoneRaw(int n_joint, const int * joints, bool * flag)
{
    return checkMotionDoneRaw(joints[0], flag);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRefSpeedRaw(int j, double sp)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d %f", j, sp);
#else
    yCTrace(IPOS, "%d %f", j, sp);
#endif
    CHECK_JOINT(j);

    if (sp < 0.0)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIWarning(IPOS, id()) << "Illegal negative speed provided:" << sp;
#else
        yCWarning(IPOS, "Illegal negative speed provided: %f", sp);
#endif
        return false;
    }
    else if (sp > vars.maxVel)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIWarning(IPOS, id()) << "Reference speed exceeds maximum velocity, i.e." << vars.maxVel.load();
#else
        yCWarning(IPOS, "Reference speed exceeds maximum velocity (%f)", vars.maxVel.load());
#endif
        return false;
    }

    double value = std::abs(vars.degreesToInternalUnits(sp, 1));

    std::uint16_t dataInt;
    std::uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    std::uint32_t data = (dataInt << 16) + dataFrac;

    if (!can->sdo()->download("Profile velocity", data, 0x6081))
    {
        return false;
    }

    vars.refSpeed = sp;
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::setRefSpeedsRaw(const double * spds)
{
    return setRefSpeedRaw(0, spds[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::setRefSpeedsRaw(int n_joint, const int * joints, const double * spds)
{
    return setRefSpeedRaw(joints[0], spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRefAccelerationRaw(int j, double acc)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d %f", j, acc);
#else
    yCTrace(IPOS, "%d %f", j, acc);
#endif
    CHECK_JOINT(j);

    if (acc < 0.0)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIWarning(IPOS, id()) << "Illegal negative acceleration provided:" << acc;
#else
        yCWarning(IPOS, "Illegal negative acceleration provided: %f", acc);
#endif
        return false;
    }

    double value = std::abs(vars.degreesToInternalUnits(acc, 2));

    std::uint16_t dataInt;
    std::uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    std::uint32_t data = (dataInt << 16) + dataFrac;

    if (!can->sdo()->download("Profile acceleration", data, 0x6083))
    {
        return false;
    }

    vars.refAcceleration = acc;
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::setRefAccelerationsRaw(const double * accs)
{
    return setRefAccelerationRaw(0, accs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs)
{
    return setRefAccelerationRaw(joints[0], accs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefSpeedRaw(int j, double * ref)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d", j);
#else
    yCTrace(IPOS, "%d", j);
#endif
    CHECK_JOINT(j);

    if (vars.actualControlMode == VOCAB_CM_NOT_CONFIGURED)
    {
        *ref = vars.refSpeed;
        return true;
    }

    return can->sdo()->upload<std::uint32_t>("Profile velocity", [this, ref](auto data)
        {
            std::uint16_t dataInt = data >> 16;
            std::uint16_t dataFrac = data & 0xFFFF;
            double value = CanUtils::decodeFixedPoint(dataInt, dataFrac);
            *ref = std::abs(vars.internalUnitsToDegrees(value, 1));
        },
        0x6081);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getRefSpeedsRaw(double * spds)
{
    return getRefSpeedRaw(0, &spds[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getRefSpeedsRaw(int n_joint, const int * joints, double * spds)
{
    return getRefSpeedRaw(joints[0], &spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefAccelerationRaw(int j, double * acc)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d", j);
#else
    yCTrace(IPOS, "%d", j);
#endif
    CHECK_JOINT(j);

    if (vars.actualControlMode == VOCAB_CM_NOT_CONFIGURED)
    {
        *acc = vars.refAcceleration;
        return true;
    }

    return can->sdo()->upload<std::uint32_t>("Profile acceleration", [this, acc](auto data)
        {
            std::uint16_t dataInt = data >> 16;
            std::uint16_t dataFrac = data & 0xFFFF;
            double value = CanUtils::decodeFixedPoint(dataInt, dataFrac);
            *acc = std::abs(vars.internalUnitsToDegrees(value, 2));
        },
        0x6083);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getRefAccelerationsRaw(double * accs)
{
    return getRefAccelerationRaw(0, &accs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getRefAccelerationsRaw(int n_joint, const int * joints, double * accs)
{
    return getRefAccelerationRaw(joints[0], &accs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::stopRaw(int j)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d", j);
#else
    yCTrace(IPOS, "%d", j);
#endif
    CHECK_JOINT(j);

    return (vars.actualControlMode == VOCAB_CM_POSITION || vars.actualControlMode == VOCAB_CM_VELOCITY)
        && can->driveStatus()->controlword(can->driveStatus()->controlword().set(8)) // stop with profile acceleration
        && (vars.synchronousCommandTarget = 0.0, true);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::stopRaw()
{
    return stopRaw(0);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::stopRaw(int n_joint, const int * joints)
{
    return stopRaw(joints[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getTargetPositionRaw(int joint, double * ref)
{
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCITrace(IPOS, id(), "%d", joint);
#else
    yCTrace(IPOS, "%d", joint);
#endif
    CHECK_JOINT(joint);

    return can->sdo()->upload<std::int32_t>("Target position", [this, ref](auto data)
        { *ref = vars.internalUnitsToDegrees(data); },
        0x607A);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getTargetPositionsRaw(double * refs)
{
    return getTargetPositionRaw(0, &refs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getTargetPositionsRaw(int n_joint, const int * joints, double * refs)
{
    return getTargetPositionRaw(joints[0], &refs[0]);
}

// --------------------------------------------------------------------------------
