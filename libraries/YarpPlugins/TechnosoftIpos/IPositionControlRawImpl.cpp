// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <bitset>

#include <ColorDebug.h>

#include "CanUtils.hpp"

using namespace roboticslab;

// --------------------------------------------------------------------------------

bool TechnosoftIpos::positionMoveRaw(int j, double ref)    // encExposed = ref;
{
    CD_DEBUG("(%d, %f)\n", j, ref);
    CHECK_JOINT(j);

    return can->rpdo1()->write<uint16_t>(0x000F) // mandatory if we call this right after the [posd->pos] transition
            && can->sdo()->download("Target position", vars.degreesToInternalUnits(ref), 0x607A)
            && can->rpdo1()->write<uint16_t>(0x003F)
            && can->rpdo1()->write<uint16_t>(0x000F); // needed to accept next target
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

    return can->rpdo1()->write<uint16_t>(0x000F) // mandatory if we call this right after the [posd->pos] transition
            && can->sdo()->download("Target position", vars.degreesToInternalUnits(delta), 0x607A)
            && can->rpdo1()->write<uint16_t>(0x007F)
            && can->rpdo1()->write<uint16_t>(0x000F); // needed to accept next target
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

    uint16_t data;

    if (!can->sdo()->upload("Statusword", &data, 0x6041))
    {
        return false;
    }

    std::bitset<16> bits(data);

    if (bits.test(0))
    {
        CD_INFO("\t-Ready to switch on. canId: %d.\n", can->getId());
    }
    if (bits.test(1))
    {
        CD_INFO("\t-Switched on. canId: %d.\n", can->getId());
    }
    if (bits.test(2))
    {
        CD_INFO("\t-Operation Enabled. canId: %d.\n", can->getId());
    }
    if (bits.test(3))
    {
        CD_INFO("\t-Fault. If set, a fault condition is or was present in the drive. canId: %d.\n", can->getId());
    }
    if (bits.test(4))
    {
        CD_INFO("\t-Motor supply voltage is present. canId: %d.\n", can->getId());
    }
    else
    {
        CD_INFO("\t-Motor supply voltage is absent. canId: %d.\n", can->getId());
    }
    if (!bits.test(5)) // negated.
    {
        CD_INFO("\t-Performing a quick stop. canId: %d.\n", can->getId());
    }
    if (bits.test(6))
    {
        CD_INFO("\t-Switch on disabled. canId: %d.\n", can->getId());
    }
    if (bits.test(7))
    {
        CD_INFO("\t-Warning. A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored. canId: %d.\n", can->getId());
    }
    if (bits.test(8))
    {
        CD_INFO("\t-A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called. canId: %d.\n", can->getId());
    }
    if (bits.test(9))
    {
        CD_INFO("\t-Remote: drive parameters may be modified via CAN and the drive will execute the command message. canId: %d.\n", can->getId());
    }
    else
    {
        CD_INFO("\t-Remote: drive is in local mode and will not execute the command message (only TML internal).");
    }
    if (bits.test(10))
    {
        CD_INFO("\t-Target reached. canId: %d.\n", can->getId());
    }
    else
    {
        CD_INFO("\t-Target not reached. canId: %d.\n", can->getId());  // improvised, not in manual, but reasonable
    }
    if (bits.test(11))
    {
        CD_INFO("\t-Internal Limit Active. canId: %d.\n", can->getId());
    }
    if (bits.test(14))
    {
        CD_INFO("\t-Last event set has ocurred. canId: %d.\n", can->getId());
    }
    else
    {
        CD_INFO("\t-No event set or the programmed event has not occurred yet. canId: %d.\n", can->getId());
    }
    if (bits.test(15))
    {
        CD_INFO("\t-Axis on. Power stage is enabled. Motor control is performed. canId: %d.\n", can->getId());
    }
    else
    {
        CD_INFO("\t-Axis off. Power stage is disabled. Motor control is not performed. canId: %d.\n", can->getId());
    }

    *flag = bits.test(10);
    return true;
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::checkMotionDoneRaw(bool * flags)
{
    CD_DEBUG("\n");
    return checkMotionDoneRaw(0, &flags[0]);
}

// --------------------------------------------------------------------------------

bool TechnosoftIpos::checkMotionDoneRaw(int n_joint, const int * joints, bool * flags)
{
    CD_DEBUG("\n");
    return checkMotionDoneRaw(joints[0], &flags[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRefSpeedRaw(int j, double sp)
{
    CD_DEBUG("(%d, %f)\n", j, sp);
    CHECK_JOINT(j);

    if (sp > vars.maxVel)
    {
        CD_WARNING("Reference speed exceeds maximum velocity (%f).\n", vars.maxVel);
        return false;
    }

    double value = std::abs(vars.degreesToInternalUnits(sp, 1));

    uint16_t dataInt;
    uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    uint32_t data = (dataInt << 16) + dataFrac;
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

    double value = std::abs(vars.degreesToInternalUnits(acc, 2));

    uint16_t dataInt;
    uint16_t dataFrac;
    CanUtils::encodeFixedPoint(value, &dataInt, &dataFrac);

    uint32_t data = (dataInt << 16) + dataFrac;
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

    return can->sdo()->upload<uint32_t>("Profile velocity", [=](uint32_t * data)
            {
                uint16_t dataInt = *data >> 16;
                uint16_t dataFrac = *data & 0xFFFF;
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

    return can->sdo()->upload<uint32_t>("Profile acceleration", [=](uint32_t * data)
            {
                uint16_t dataInt = *data >> 16;
                uint16_t dataFrac = *data & 0xFFFF;
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

    return can->sdo()->upload<int32_t>("Target position", [=](int32_t * data)
            { *ref = vars.internalUnitsToDegrees(*data); },
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
