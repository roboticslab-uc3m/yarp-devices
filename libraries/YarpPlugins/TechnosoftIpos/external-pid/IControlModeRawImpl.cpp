// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getControlModeRaw(int j, int * mode)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    *mode = actualControlMode;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setControlModeRaw(int j, int mode)
{
    yCITrace(IPOS, id(), "%d %s", j, yarp::os::Vocab32::decode(mode).c_str());
    CHECK_JOINT(j);

    requestedcontrolMode = mode;

    if (mode == actualControlMode)
    {
        return true;
    }

    switch (mode)
    {
    case VOCAB_CM_POSITION:
    case VOCAB_CM_VELOCITY:
        trajectory.reset(internalUnitsToDegrees(lastEncoderRead->queryPosition()));
        resetPidRaw(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, 0);
        break;
    case VOCAB_CM_POSITION_DIRECT:
        commandBuffer.reset(internalUnitsToDegrees(lastEncoderRead->queryPosition()));
        break;
    case VOCAB_CM_TORQUE:
    case VOCAB_CM_CURRENT:
        commandBuffer.reset(0.0);
        break;
    case VOCAB_CM_FORCE_IDLE:
        if (actualControlMode == VOCAB_CM_HW_FAULT && !can->driveStatus()->requestTransition(DriveTransition::FAULT_RESET))
        {
            yCIError(IPOS, id()) << "Unable to reset fault status";
            return false;
        }
        // no break
    case VOCAB_CM_IDLE:
        return can->driveStatus()->requestState(DriveState::SWITCHED_ON)
            && can->sdo()->download<std::int8_t>("Modes of Operation", 0, 0x6060) // reset drive mode
            && can->driveStatus()->controlword(can->driveStatus()->controlword().reset(4)); // disable ext. ref. torque mode
    default:
        yCIError(IPOS, id()) << "Unsupported, unknown or read-only mode:" << yarp::os::Vocab32::decode(mode);
        return false;
    }

    switch (actualControlMode)
    {
    case VOCAB_CM_POSITION:
    case VOCAB_CM_VELOCITY:
    case VOCAB_CM_POSITION_DIRECT:
    case VOCAB_CM_TORQUE:
    case VOCAB_CM_CURRENT:
        actualControlMode = mode;
        return true;
    default:
        PdoConfiguration rpdo3conf;
        rpdo3conf.setTransmissionType(PdoTransmissionType::SYNCHRONOUS_CYCLIC);

        return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
            && can->rpdo3()->configure(rpdo3conf.addMapping<std::int32_t>(0x201C))
            && can->sdo()->download<std::uint16_t>("External Reference Type", 1, 0x201D)
            && can->sdo()->download<std::int8_t>("Modes of Operation", -5, 0x6060)
            // configure new setpoint (4: enable ext. ref. torque mode), reset other mode-specific bits (5-6) and halt bit (8)
            && can->driveStatus()->controlword(can->driveStatus()->controlword().set(4).reset(5).reset(6).reset(8))
            && awaitControlMode(mode);
    }
}

// -----------------------------------------------------------------------------
