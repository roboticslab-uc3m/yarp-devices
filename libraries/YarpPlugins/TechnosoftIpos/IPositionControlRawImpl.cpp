// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cmath>

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

// --------------------------------------------------------------------------------

bool TechnosoftIpos::positionMoveRaw(int j, double ref)
{
    CD_DEBUG("(%d, %f)\n", j, ref);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    return can->rpdo1()->write<std::uint16_t>(0x000F) // mandatory if we call this right after the [posd->pos] transition
            && can->sdo()->download<std::int32_t>("Target position", vars.degreesToInternalUnits(ref), 0x607A)
            && can->rpdo1()->write<std::uint16_t>(0x003F)
            && can->rpdo1()->write<std::uint16_t>(0x000F); // needed to accept next target
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::positionMoveRaw(const double * refs)
{
    CD_DEBUG("\n");
    return positionMoveRaw(0, refs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::positionMoveRaw(int n_joint, const int * joints, const double * refs)
{
    CD_DEBUG("\n");
    return positionMoveRaw(joints[0], refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::relativeMoveRaw(int j, double delta)
{
    CD_DEBUG("(%d, %f)\n", j, delta);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    return can->rpdo1()->write<std::uint16_t>(0x000F) // mandatory if we call this right after the [posd->pos] transition
            && can->sdo()->download<std::int32_t>("Target position", vars.degreesToInternalUnits(delta), 0x607A)
            && can->rpdo1()->write<std::uint16_t>(0x007F)
            && can->rpdo1()->write<std::uint16_t>(0x000F); // needed to accept next target
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::relativeMoveRaw(const double * deltas)
{
    CD_DEBUG("\n");
    return relativeMoveRaw(0, deltas[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::relativeMoveRaw(int n_joint, const int * joints, const double * deltas)
{
    CD_DEBUG("\n");
    return relativeMoveRaw(joints[0], deltas[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::checkMotionDoneRaw(int j, bool * flag)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    *flag = can->driveStatus()->getCurrentState() != DriveState::OPERATION_ENABLED || can->driveStatus()->statusword()[10];
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::checkMotionDoneRaw(bool * flag)
{
    CD_DEBUG("\n");
    return checkMotionDoneRaw(0, flag);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::checkMotionDoneRaw(int n_joint, const int * joints, bool * flag)
{
    CD_DEBUG("\n");
    return checkMotionDoneRaw(joints[0], flag);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRefSpeedRaw(int j, double sp)
{
    CD_DEBUG("(%d, %f)\n", j, sp);
    CHECK_JOINT(j);

    if (sp < 0.0)
    {
        CD_WARNING("Illegal negative speed provided: %f.\n");
        return false;
    }
    else if (sp > vars.maxVel)
    {
        CD_WARNING("Reference speed exceeds maximum velocity (%f).\n", vars.maxVel);
        return false;
    }

    double value = std::abs(vars.degreesToInternalUnits(sp, 1));

    std::uint16_t dataInt;
    std::uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    std::uint32_t data = (dataInt << 16) + dataFrac;
    return can->sdo()->download("Profile velocity", data, 0x6081);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::setRefSpeedsRaw(const double * spds)
{
    CD_DEBUG("\n");
    return setRefSpeedRaw(0, spds[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::setRefSpeedsRaw(int n_joint, const int * joints, const double * spds)
{
    CD_DEBUG("\n");
    return setRefSpeedRaw(joints[0], spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRefAccelerationRaw(int j, double acc)
{
    CD_DEBUG("(%d, %f)\n", j, acc);
    CHECK_JOINT(j);

    if (acc < 0.0)
    {
        CD_WARNING("Illegal negative acceleration provided: %f.\n");
        return false;
    }

    double value = std::abs(vars.degreesToInternalUnits(acc, 2));

    std::uint16_t dataInt;
    std::uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    std::uint32_t data = (dataInt << 16) + dataFrac;
    return can->sdo()->download("Profile acceleration", data, 0x6083);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::setRefAccelerationsRaw(const double * accs)
{
    CD_DEBUG("\n");
    return setRefAccelerationRaw(0, accs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs)
{
    CD_DEBUG("\n");
    return setRefAccelerationRaw(joints[0], accs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefSpeedRaw(int j, double * ref)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    return can->sdo()->upload<std::uint32_t>("Profile velocity", [&](std::uint32_t data)
            {
                std::uint16_t dataInt = data >> 16;
                std::uint16_t dataFrac = data & 0xFFFF;
                double value = CanUtils::decodeFixedPoint(dataInt, dataFrac);
                *ref = vars.internalUnitsToDegrees(value, 1);
            },
            0x6081);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getRefSpeedsRaw(double * spds)
{
    CD_DEBUG("\n");
    return getRefSpeedRaw(0, &spds[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getRefSpeedsRaw(int n_joint, const int * joints, double * spds)
{
    CD_DEBUG("\n");
    return getRefSpeedRaw(joints[0], &spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefAccelerationRaw(int j, double * acc)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    return can->sdo()->upload<std::uint32_t>("Profile acceleration", [&](std::uint32_t data)
            {
                std::uint16_t dataInt = data >> 16;
                std::uint16_t dataFrac = data & 0xFFFF;
                double value = CanUtils::decodeFixedPoint(dataInt, dataFrac);
                *acc = vars.internalUnitsToDegrees(value, 2);
            },
            0x6083);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getRefAccelerationsRaw(double * accs)
{
    CD_DEBUG("\n");
    return getRefAccelerationRaw(0, &accs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getRefAccelerationsRaw(int n_joint, const int * joints, double * accs)
{
    CD_DEBUG("\n");
    return getRefAccelerationRaw(joints[0], &accs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::stopRaw(int j)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    return can->driveStatus()->requestTransition(DriveTransition::QUICK_STOP)
            && can->driveStatus()->requestTransition(DriveTransition::ENABLE_OPERATION);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::stopRaw()
{
    CD_DEBUG("\n");
    return stopRaw(0);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::stopRaw(int n_joint, const int * joints)
{
    CD_DEBUG("\n");
    return stopRaw(joints[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getTargetPositionRaw(int joint, double * ref)
{
    CD_DEBUG("\n");
    CHECK_JOINT(joint);

    return can->sdo()->upload<std::int32_t>("Target position", [&](std::int32_t data)
            { *ref = vars.internalUnitsToDegrees(data); },
            0x607A);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getTargetPositionsRaw(double * refs)
{
    CD_DEBUG("\n");
    return getTargetPositionRaw(0, &refs[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::getTargetPositionsRaw(int n_joint, const int * joints, double * refs)
{
    CD_DEBUG("\n");
    return getTargetPositionRaw(joints[0], &refs[0]);
}

// --------------------------------------------------------------------------------
