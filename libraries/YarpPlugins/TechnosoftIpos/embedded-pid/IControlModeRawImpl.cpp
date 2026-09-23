// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::getAvailableControlModesRaw(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail)
{
    CHECK_JOINT(j);

    avail = {
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_VELOCITY,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_CURRENT,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_TORQUE,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION_DIRECT,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_FORCE_IDLE,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE
    };

    return yarp::dev::ReturnValue_ok;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::getControlModeRaw(int j, yarp::dev::ControlModeEnum & mode)
#else
bool TechnosoftIposEmbedded::getControlModeRaw(int j, int * mode)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    mode = static_cast<yarp::dev::ControlModeEnum>(actualControlMode.load());
    return yarp::dev::ReturnValue_ok;
#else
    *mode = actualControlMode;
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::setControlModeRaw(int j, yarp::dev::SelectableControlModeEnum mode)
#else
bool TechnosoftIposEmbedded::setControlModeRaw(int j, int mode)
#endif
{
    CHECK_JOINT(j);

    const auto modeVocab = static_cast<yarp::conf::vocab32_t>(mode);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    requestedcontrolMode = modeVocab;
#else
    requestedcontrolMode = mode;
#endif
    bool extRefTorque = actualControlMode == VOCAB_CM_TORQUE || actualControlMode == VOCAB_CM_CURRENT;

    if (modeVocab == actualControlMode || (extRefTorque && (modeVocab == VOCAB_CM_CURRENT || modeVocab == VOCAB_CM_TORQUE)))
    {
        actualControlMode.store(requestedcontrolMode); // disambiguate torque/current modes
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_ok;
#else
        return true;
#endif
    }

    enableSync = false;

    // reset mode-specific bits (4-6) and halt bit (8)
    if (!can->driveStatus()->controlword(can->driveStatus()->controlword().reset(4).reset(5).reset(6).reset(8)))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    // bug in F508M/F509M firmware, switch to homing mode to stop controlling external reference torque
    if (extRefTorque && !can->sdo()->download<std::int8_t>("Modes of Operation", 6, 0x6060))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    PdoConfiguration rpdo3conf;
    rpdo3conf.setTransmissionType(PdoTransmissionType::SYNCHRONOUS_CYCLIC);

    switch (modeVocab)
    {
    case VOCAB_CM_POSITION:
        targetPosition = internalUnitsToDegrees(lastEncoderRead->queryPosition());

        return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
            && can->sdo()->download<std::int32_t>("Target position", lastEncoderRead->queryPosition(), 0x607A)
            && can->sdo()->download<std::int8_t>("Modes of Operation", 1, 0x6060)
            && can->driveStatus()->controlword(can->driveStatus()->controlword().set(5)) // change set immediately
            && awaitControlMode(modeVocab)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
            ;
#endif

    case VOCAB_CM_VELOCITY:
        if (enableCsv)
        {
            commandBuffer.reset(0.0);

            return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
                && can->rpdo3()->configure(rpdo3conf.addMapping<std::int32_t>(0x607A))
                && can->sdo()->download<std::uint8_t>("Interpolation time period", params.m_syncPeriod * 1000, 0x60C2, 0x01)
                && can->sdo()->download<std::int8_t>("Interpolation time period", -3, 0x60C2, 0x02)
                && can->sdo()->download<std::int8_t>("Modes of Operation", 8, 0x6060)
                && can->driveStatus()->controlword(can->driveStatus()->controlword().set(6)) // relative position mode
                && awaitControlMode(modeVocab)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
                ;
#endif
        }
        else
        {
            targetVelocity = 0.0;

            return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
                && can->sdo()->download<std::int8_t>("Modes of Operation", 3, 0x6060)
                && awaitControlMode(modeVocab)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
                ;
#endif
        }

    case VOCAB_CM_CURRENT:
    case VOCAB_CM_TORQUE:
        commandBuffer.reset(0.0);

        return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
            && can->rpdo3()->configure(rpdo3conf.addMapping<std::int32_t>(0x201C))
            && can->sdo()->download<std::uint16_t>("External Reference Type", 1, 0x201D)
            && can->sdo()->download<std::int8_t>("Modes of Operation", -5, 0x6060)
            && can->driveStatus()->controlword(can->driveStatus()->controlword().set(4)) // enable ext. ref. torque mode
            && awaitControlMode(modeVocab)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
            ;
#endif

    case VOCAB_CM_POSITION_DIRECT:
        if (ipBuffer)
        {
            ipBufferFilled = ipMotionStarted = false;
            ipBufferEnabled = true;
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
                && awaitControlMode(modeVocab)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
                ;
#endif
        }

        // bug in F508M/F509M firmware, switch to homing mode to stop controlling profile velocity
        if (actualControlMode == VOCAB_CM_VELOCITY && !enableCsv
            && !can->sdo()->download<std::int8_t>("Modes of Operation", 6, 0x6060))
        {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue_error_method_failed;
#else
            return false;
#endif
        }

        commandBuffer.reset(internalUnitsToDegrees(lastEncoderRead->queryPosition()));

        return can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
            && can->rpdo3()->configure(rpdo3conf.addMapping<std::int32_t>(0x607A))
            && can->sdo()->download<std::uint8_t>("Interpolation time period", params.m_syncPeriod * 1000, 0x60C2, 0x01)
            && can->sdo()->download<std::int8_t>("Interpolation time period", -3, 0x60C2, 0x02)
            && can->sdo()->download<std::int8_t>("Modes of Operation", 8, 0x6060)
            && awaitControlMode(modeVocab)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
            ;
#endif

    case VOCAB_CM_FORCE_IDLE:
        if (actualControlMode == VOCAB_CM_HW_FAULT
            && !can->driveStatus()->requestTransition(DriveTransition::FAULT_RESET))
        {
            yCIError(IPOS, id()) << "Unable to reset fault status";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue_error_method_failed;
#else
            return false;
#endif
        }

        // no break

    case VOCAB_CM_IDLE:
        return can->driveStatus()->requestState(DriveState::SWITCHED_ON)
            && can->sdo()->download<std::int8_t>("Modes of Operation", 0, 0x6060) // reset drive mode
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
            ;
#endif

    default:
        yCIError(IPOS, id()) << "Unsupported, unknown or read-only mode:" << yarp::os::Vocab32::decode(modeVocab);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }
}

// -----------------------------------------------------------------------------
