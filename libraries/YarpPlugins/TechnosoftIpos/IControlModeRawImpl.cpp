// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModeRaw(int j, int * mode)
{
#if YARP_VERSION_MINOR >= 6
    yCITrace(IPOS, id(), "%d", j);
#else
    yCTrace(IPOS, "%d", j);
#endif
    CHECK_JOINT(j);
    *mode = vars.actualControlMode;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModesRaw(int * modes)
{
    return getControlModeRaw(0, &modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getControlModesRaw(int n_joint, const int * joints, int * modes)
{
    return getControlModeRaw(joints[0], &modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setControlModeRaw(int j, int mode)
{
#if YARP_VERSION_MINOR >= 6
    yCITrace(IPOS, id(), "%d %s", j, yarp::os::Vocab32::decode(mode).c_str());
#elif YARP_VERSION_MINOR >= 5
    yCTrace(IPOS, "%d %s", j, yarp::os::Vocab32::decode(mode).c_str());
#else
    yCTrace(IPOS, "%d %s", j, yarp::os::Vocab::decode(mode).c_str());
#endif
    CHECK_JOINT(j);

    vars.requestedcontrolMode = mode;
    bool extRefTorque = vars.actualControlMode == VOCAB_CM_TORQUE || vars.actualControlMode == VOCAB_CM_CURRENT;

    if (mode == vars.actualControlMode || (extRefTorque && (mode == VOCAB_CM_CURRENT || mode == VOCAB_CM_TORQUE)))
    {
        vars.actualControlMode.store(vars.requestedcontrolMode.load()); // disambiguate torque/current modes
        return true;
    }

    vars.enableSync = false;

    // reset mode-specific bits (4-6) and halt bit (8)
    if (!can->driveStatus()->controlword(can->driveStatus()->controlword().reset(4).reset(5).reset(6).reset(8)))
    {
        return false;
    }

    // bug in F508M/F509M firmware, switch to homing mode to stop controlling external reference torque
    if (extRefTorque && !can->sdo()->download<std::int8_t>("Modes of Operation", 6, 0x6060))
    {
        return false;
    }

    PdoConfiguration rpdo3conf;
    rpdo3conf.setTransmissionType(PdoTransmissionType::SYNCHRONOUS_CYCLIC);

    switch (mode)
    {
    case VOCAB_CM_POSITION:
        return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
            && can->sdo()->download<std::int32_t>("Target position", vars.lastEncoderRead->queryPosition(), 0x607A)
            && can->sdo()->download<std::int8_t>("Modes of Operation", 1, 0x6060)
            && can->driveStatus()->controlword(can->driveStatus()->controlword().set(5)) // change set immediately
            && vars.awaitControlMode(mode);

    case VOCAB_CM_VELOCITY:
        vars.synchronousCommandTarget = 0.0;

        if (vars.enableCsv)
        {
            return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
                && can->rpdo3()->configure(rpdo3conf.addMapping<std::int32_t>(0x607A))
                && can->sdo()->download<std::uint8_t>("Interpolation time period", vars.syncPeriod * 1000, 0x60C2, 0x01)
                && can->sdo()->download<std::int8_t>("Interpolation time period", -3, 0x60C2, 0x02)
                && can->sdo()->download<std::int8_t>("Modes of Operation", 8, 0x6060)
                && can->driveStatus()->controlword(can->driveStatus()->controlword().set(6)) // relative position mode
                && vars.awaitControlMode(mode);
        }
        else
        {
            return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
                && can->rpdo3()->configure(rpdo3conf.addMapping<std::int32_t>(0x60FF))
                && can->sdo()->download<std::int8_t>("Modes of Operation", 3, 0x6060)
                && vars.awaitControlMode(mode);
        }

    case VOCAB_CM_CURRENT:
    case VOCAB_CM_TORQUE:
        vars.synchronousCommandTarget = 0.0;

        return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
            && can->rpdo3()->configure(rpdo3conf.addMapping<std::int32_t>(0x201C))
            && can->sdo()->download<std::uint16_t>("External Reference Type", 1, 0x201D)
            && can->sdo()->download<std::int8_t>("Modes of Operation", -5, 0x6060)
            && can->driveStatus()->controlword(can->driveStatus()->controlword().set(4)) // new setpoint (assume target position)
            && vars.awaitControlMode(mode);

    case VOCAB_CM_POSITION_DIRECT:
        if (ipBuffer)
        {
            vars.ipBufferFilled = vars.ipMotionStarted = false;
            vars.ipBufferEnabled = true;
            ipBuffer->clearQueue();

            PdoConfiguration rpdo3Conf;
            rpdo3Conf.addMapping<std::uint32_t>(0x60C1, 0x01);
            rpdo3Conf.addMapping<std::uint32_t>(0x60C1, 0x02);

            return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
                && can->rpdo3()->configure(rpdo3Conf)
                && can->sdo()->download<std::uint16_t>("Auxiliary Settings Register", 0x0000, 0x208E) // legacy ip mode
                && can->sdo()->download("Interpolation sub mode select", ipBuffer->getSubMode(), 0x60C0)
                && can->sdo()->download("Interpolated position buffer length", ipBuffer->getBufferSize(), 0x2073)
                && can->sdo()->download("Interpolated position buffer configuration", ipBuffer->getBufferConfig(), 0x2074)
                && can->sdo()->download<std::int8_t>("Modes of Operation", 7, 0x6060)
                && vars.awaitControlMode(VOCAB_CM_POSITION_DIRECT);
        }

        // bug in F508M/F509M firmware, switch to homing mode to stop controlling profile velocity
        if (vars.actualControlMode == VOCAB_CM_VELOCITY && !vars.enableCsv
            && !can->sdo()->download<std::int8_t>("Modes of Operation", 6, 0x6060))
        {
            return false;
        }

        vars.synchronousCommandTarget = vars.internalUnitsToDegrees(vars.lastEncoderRead->queryPosition());
        vars.prevSyncTarget.store(vars.synchronousCommandTarget);

        return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
            && can->rpdo3()->configure(rpdo3conf.addMapping<std::int32_t>(0x607A))
            && can->sdo()->download<std::uint8_t>("Interpolation time period", vars.syncPeriod * 1000, 0x60C2, 0x01)
            && can->sdo()->download<std::int8_t>("Interpolation time period", -3, 0x60C2, 0x02)
            && can->sdo()->download<std::int8_t>("Modes of Operation", 8, 0x6060)
            && vars.awaitControlMode(mode);

    case VOCAB_CM_FORCE_IDLE:
        if (vars.actualControlMode == VOCAB_CM_HW_FAULT
            && !can->driveStatus()->requestTransition(DriveTransition::FAULT_RESET))
        {
#if YARP_VERSION_MINOR >= 6
            yCIError(IPOS, id()) << "Unable to reset fault status";
#else
            yCError(IPOS, "Unable to reset fault status");
#endif
            return false;
        }

        // no break

    case VOCAB_CM_IDLE:
        return can->driveStatus()->requestState(DriveState::SWITCHED_ON)
            && can->sdo()->download<std::int8_t>("Modes of Operation", 0, 0x6060); // reset drive mode

    default:
#if YARP_VERSION_MINOR >= 6
        yCIError(IPOS, id()) << "Unsupported, unknown or read-only mode:" << yarp::os::Vocab32::decode(mode);
#elif YARP_VERSION_MINOR >= 5
        yCError(IPOS, "Unsupported, unknown or read-only mode: %s", yarp::os::Vocab32::decode(mode).c_str());
#else
        yCError(IPOS, "Unsupported, unknown or read-only mode: %s", yarp::os::Vocab::decode(mode).c_str());
#endif
        return false;
    }
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setControlModesRaw(int * modes)
{
    return setControlModeRaw(0, modes[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setControlModesRaw(int n_joint, const int * joints, int * modes)
{
    return setControlModeRaw(joints[0], modes[0]);
}

// -----------------------------------------------------------------------------
