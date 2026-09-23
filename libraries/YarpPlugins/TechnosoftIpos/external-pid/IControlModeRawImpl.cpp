// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getAvailableControlModesRaw(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail)
{
    CHECK_JOINT(j);

    avail = {
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_VELOCITY,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION_DIRECT,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_TORQUE,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_CURRENT,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_FORCE_IDLE,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_IDLE
    };

    return yarp::dev::ReturnValue::return_code::return_value_ok;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getControlModeRaw(int j, yarp::dev::ControlModeEnum & mode)
#else
bool TechnosoftIposExternal::getControlModeRaw(int j, int * mode)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    mode = static_cast<yarp::dev::ControlModeEnum>(actualControlMode.load());
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    *mode = actualControlMode;
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::setControlModeRaw(int j, yarp::dev::SelectableControlModeEnum mode)
#else
bool TechnosoftIposExternal::setControlModeRaw(int j, int mode)
#endif
{
    CHECK_JOINT(j);

    auto modeVocab = static_cast<yarp::conf::vocab32_t>(mode);
    requestedcontrolMode = modeVocab;

    if (modeVocab == actualControlMode)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
        return true;
#endif
    }

    switch (modeVocab)
    {
    case VOCAB_CM_POSITION:
    case VOCAB_CM_VELOCITY:
    case VOCAB_CM_POSITION_DIRECT:
        if (modeVocab == VOCAB_CM_POSITION || modeVocab == VOCAB_CM_VELOCITY && !enableCsv)
        {
            trajectory.reset(internalUnitsToDegrees(lastEncoderRead->queryPosition()));
        }
        else if (modeVocab == VOCAB_CM_POSITION_DIRECT)
        {
            commandBuffer.reset(internalUnitsToDegrees(lastEncoderRead->queryPosition()));
        }
        else // modeVocab == VOCAB_CM_VELOCITY && enableCsv
        {
            commandBuffer.reset(0.0);
        }

        resetPidRaw(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, j);
        break;
    case VOCAB_CM_TORQUE:
    case VOCAB_CM_CURRENT:
        commandBuffer.reset(0.0);
        break;
    case VOCAB_CM_FORCE_IDLE:
        if (actualControlMode == VOCAB_CM_HW_FAULT && !can->driveStatus()->requestTransition(DriveTransition::FAULT_RESET))
        {
            yCIError(IPOS, id()) << "Unable to reset fault status";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
            return false;
#endif
        }
        // no break
    case VOCAB_CM_IDLE:
        return can->driveStatus()->requestState(DriveState::SWITCHED_ON)
            && can->sdo()->download<std::int8_t>("Modes of Operation", 0, 0x6060) // reset drive mode
            && can->driveStatus()->controlword(can->driveStatus()->controlword().reset(4)) // disable ext. ref. torque mode
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            ? yarp::dev::ReturnValue::return_code::return_value_ok
            : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
            ;
#endif
    default:
        yCIError(IPOS, id()) << "Unsupported, unknown or read-only mode:" << yarp::os::Vocab32::decode(modeVocab);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    switch (actualControlMode)
    {
    case VOCAB_CM_POSITION:
    case VOCAB_CM_VELOCITY:
    case VOCAB_CM_POSITION_DIRECT:
    case VOCAB_CM_TORQUE:
    case VOCAB_CM_CURRENT:
        actualControlMode = modeVocab;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
        return true;
#endif
    default:
        PdoConfiguration rpdo3conf;
        rpdo3conf.setTransmissionType(PdoTransmissionType::SYNCHRONOUS_CYCLIC);

        bool ret = can->driveStatus()->requestState(DriveState::OPERATION_ENABLED)
            && can->rpdo3()->configure(rpdo3conf.addMapping<std::int32_t>(0x201C))
            && can->sdo()->download<std::uint16_t>("External Reference Type", 1, 0x201D)
            && can->sdo()->download<std::int8_t>("Modes of Operation", -5, 0x6060)
            // configure new setpoint (4: enable ext. ref. torque mode), reset other mode-specific bits (5-6) and halt bit (8)
            && can->driveStatus()->controlword(can->driveStatus()->controlword().set(4).reset(5).reset(6).reset(8))
            && awaitControlMode(modeVocab);

        // the point of the following instructions is to refresh the position reference as close to the command loop (in
        // synchronize()) as possible; without this, the motor may jolt right after the transition from idle to command mode

        if (ret) // successfully updated `actualControlMode`
        {
            if (modeVocab == VOCAB_CM_POSITION || modeVocab == VOCAB_CM_VELOCITY)
            {
                trajectory.reset(internalUnitsToDegrees(lastEncoderRead->queryPosition()));
            }
            else if (modeVocab == VOCAB_CM_POSITION_DIRECT)
            {
                commandBuffer.reset(internalUnitsToDegrees(lastEncoderRead->queryPosition()));
            }
        }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return ret ? yarp::dev::ReturnValue::return_code::return_value_ok
                   : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return ret;
#endif
    }
}

// -----------------------------------------------------------------------------
