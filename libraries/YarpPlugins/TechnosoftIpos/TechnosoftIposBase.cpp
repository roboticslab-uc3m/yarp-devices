// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <cstring>

#include <sstream>
#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::reportBitToggle(report_storage report, int level, std::size_t pos, const char * msgSet, const char * msgReset)
{
    bool isSet = report.actual.test(pos);

    if (report.stored.test(pos) != isSet && level != NONE)
    {
        std::stringstream ss;
        ss << "[" << report.reg << "] ";

        if (isSet)
        {
            ss << msgSet;
        }
        else
        {
            ss << (!msgReset ? std::string("Bit reset: ") + msgSet : msgReset);
        }

        if (isSet && level != INFO)
        {
            yCIWarning(IPOS, id(), "%s", ss.str().c_str());

            if (level == FAULT)
            {
                vars.lastFaultMessage = msgSet;
            }
        }
        else
        {
            yCIInfo(IPOS, id(), "%s", ss.str().c_str());
        }
    }

    return isSet;
}

// -----------------------------------------------------------------------------

void TechnosoftIposBase::interpretMsr(std::uint16_t msr)
{
    report_storage report{"MSR", msr, vars.msr};

    reportBitToggle(report, INFO, 0, "Drive/motor initialization performed.");
    reportBitToggle(report, NONE, 1, "Position trigger 1 reached.");
    reportBitToggle(report, NONE, 2, "Position trigger 2 reached.");
    reportBitToggle(report, NONE, 3, "Position trigger 3 reached.");
    reportBitToggle(report, NONE, 4, "Position trigger 4 reached.");
    reportBitToggle(report, INFO, 5, "AUTORUN mode enabled.");
    reportBitToggle(report, INFO, 6, "Limit switch positive event / interrupt triggered.");
    reportBitToggle(report, INFO, 7, "Limit switch negative event / interrupt triggered.");
    reportBitToggle(report, INFO, 8, "Capture event/interrupt triggered.");
    reportBitToggle(report, INFO, 9, "Target command reached.");
    reportBitToggle(report, WARN, 10, "Motor I2t protection warning level reached.");
    reportBitToggle(report, WARN, 11, "Drive I2t protection warning level reached.");
    reportBitToggle(report, INFO, 12, "Gear ratio in electronic gearing mode reached.");
    // 13 (29): reserved
    reportBitToggle(report, INFO, 14, "Reference position in absolute electronic camming mode reached.");
    reportBitToggle(report, WARN, 15, "Drive/motor in fault status.");

    vars.msr = msr;
}

// -----------------------------------------------------------------------------

void TechnosoftIposBase::interpretMer(std::uint16_t mer)
{
    report_storage report{"MER", mer, vars.mer};

    bool isSet = false;

    isSet |= reportBitToggle(report, FAULT, 0, "CAN error.");
    isSet |= reportBitToggle(report, FAULT, 1, "Short-circuit.");
    isSet |= reportBitToggle(report, FAULT, 2, "Invalid setup data.");
    isSet |= reportBitToggle(report, FAULT, 3, "Control error (position/speed error too big).");
    isSet |= reportBitToggle(report, FAULT, 4, "Communication error.");
    isSet |= reportBitToggle(report, FAULT, 5, "Motor position wraps around.");
    isSet |= reportBitToggle(report, FAULT, 6, "Positive limit switch active.");
    isSet |= reportBitToggle(report, FAULT, 7, "Negative limit switch active.");
    isSet |= reportBitToggle(report, FAULT, 8, "Over current.");
    isSet |= reportBitToggle(report, FAULT, 9, "I2t protection.");
    isSet |= reportBitToggle(report, FAULT, 10, "Over temperature motor.");
    isSet |= reportBitToggle(report, FAULT, 11, "Over temperature drive.");
    isSet |= reportBitToggle(report, FAULT, 12, "Over-voltage.");
    isSet |= reportBitToggle(report, FAULT, 13, "Under-voltage.");
    isSet |= reportBitToggle(report, FAULT, 14, "Command error.");
    isSet |= reportBitToggle(report, FAULT, 15, "Drive disabled due to enable or STO input.");

    if (isSet)
    {
        vars.lastFaultCode = mer;
    }

    vars.mer = mer;
}

// -----------------------------------------------------------------------------

void TechnosoftIposBase::interpretDer(std::uint16_t der)
{
    report_storage report{"DER", der, vars.der};

    reportBitToggle(report, WARN, 0, "The number of nested function calls exceeded the length of TML stack. Last function call was ignored.");
    reportBitToggle(report, WARN, 1, "A RET/RETI instruction was executed while no function/ISR was active.");
    reportBitToggle(report, WARN, 2, "A call to an inexistent homing routine was received.");
    reportBitToggle(report, WARN, 3, "A call to an inexistent function was received.");
    reportBitToggle(report, WARN, 4, "UPD instruction received while AXISON was executed. The UPD instruction was ignored and it must be sent again when AXISON is completed.");
    reportBitToggle(report, WARN, 5, "Cancelable call instruction received while another cancelable function was active.");
    reportBitToggle(report, WARN, 6, "Positive software limit switch is active.");
    reportBitToggle(report, WARN, 7, "Negative software limit switch is active.");
    reportBitToggle(report, WARN, 8, "S-curve parameters caused and invalid profile. UPD instruction was ignored.");
    reportBitToggle(report, WARN, 9, "Update ignored for S-curve.");
    reportBitToggle(report, WARN, 10, "Encoder broken wire; On a brushless motor, either the digital halls or the incremental encoder signal was interrupted.");
    reportBitToggle(report, WARN, 11, "Start mode failed. Motionless start or pole lock minimum movement failed.");
    // 12: reserved
    reportBitToggle(report, WARN, 13, "Self-check error. Internal memory (OTP) checksum error.");
    reportBitToggle(report, WARN, 14, "STO or Enable circuit hardware error.");
    reportBitToggle(report, WARN, 15, "EEPROM Locked. An attempt to write in the EEPROM will be ignored.");

    vars.der = der;
}

// -----------------------------------------------------------------------------

void TechnosoftIposBase::interpretDer2(std::uint16_t der2)
{
    report_storage report{"DER2", der2, vars.der2};

    reportBitToggle(report, WARN, 0, "BiSS data CRC error");
    reportBitToggle(report, WARN, 1, "BiSS data warning bit is set");
    reportBitToggle(report, WARN, 2, "BiSS data error bit is set");
    reportBitToggle(report, WARN, 3, "BiSS sensor missing. No BiSS sensor communication detected.");
    reportBitToggle(report, WARN, 4, "Absolute Encoder Interface (AEI) interface error.");
    reportBitToggle(report, WARN, 5, "Hall sensor missing. Can be either Digital or Linear analogue hall error.");
    reportBitToggle(report, WARN, 6, "Position wraparound.");
    // 6-15: reserved

    vars.der2 = der2;
}

// -----------------------------------------------------------------------------

void TechnosoftIposBase::interpretCer(std::uint16_t cer)
{
    report_storage report{"CER", cer, vars.cer};

    reportBitToggle(report, WARN, 0, "RS232 reception error.");
    reportBitToggle(report, WARN, 1, "RS232 transmission timeout error.");
    reportBitToggle(report, WARN, 2, "RS232 reception timeout error.");
    reportBitToggle(report, WARN, 3, "CAN reception timeout error.");
    reportBitToggle(report, WARN, 4, "CAN reception overrun error.");
    reportBitToggle(report, WARN, 5, "CAN transmission overrun error.");
    reportBitToggle(report, WARN, 6, "CAN bus off error.");
    reportBitToggle(report, WARN, 7, "SPI timeout on write operation.");
    // 8-15: reserved

    vars.cer = cer;
}

// -----------------------------------------------------------------------------

void TechnosoftIposBase::interpretStatusword(std::uint16_t statusword)
{
    switch (DriveStatusMachine::parseStatusword(statusword))
    {
    case DriveState::FAULT_REACTION_ACTIVE:
    case DriveState::FAULT:
        vars.actualControlMode = VOCAB_CM_HW_FAULT;
        break;
    case DriveState::SWITCHED_ON:
        vars.actualControlMode = VOCAB_CM_IDLE;
        break;
    default:
        break;
    }

    report_storage report{"status", statusword, can->driveStatus()->statusword()};

    reportBitToggle(report, INFO, 0, "Ready to switch on.");
    reportBitToggle(report, INFO, 1, "Switched on.");
    reportBitToggle(report, INFO, 2, "Operation enabled.");
    reportBitToggle(report, INFO, 3, "Fault.");
    reportBitToggle(report, INFO, 4, "Motor supply voltage is present.", "Motor supply voltage is absent");
    reportBitToggle(report, INFO, 5, "Quick Stop.");
    reportBitToggle(report, INFO, 6, "Switch on disabled.");
    reportBitToggle(report, WARN, 7, "A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored.",
            "No warning.");
    reportBitToggle(report, INFO, 8, "A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called.",
            "No TML function or homing is executed. The execution of the last called TML function or homing is completed.");
    reportBitToggle(report, INFO, 9, "Remote - drive parameters may be modified via CAN and the drive will execute the command message.",
            "Remote – drive is in local mode and will not execute the command message.");

    if (reportBitToggle(report, INFO, 10, "Target reached.")
        && vars.actualControlMode == VOCAB_CM_POSITION // does not work on velocity profile mode
        && can->driveStatus()->controlword()[8]
        && !can->driveStatus()->controlword(can->driveStatus()->controlword().reset(8)))
    {
        yCIWarning(IPOS, id()) << "Unable to reset halt bit";
    }

    reportBitToggle(report, INFO, 11, "Internal Limit Active.");

    switch (vars.modesOfOperation)
    {
    case 1: // profile position
        if (reportBitToggle(report, INFO, 12, "Trajectory generator will not accept a new set-point.",
            "Trajectory generator will accept a new set-point.")
            && !can->driveStatus()->controlword(can->driveStatus()->controlword().reset(4)))
        {
            yCIWarning(IPOS, id()) << "Unable to finalize single set-point handshake";
        }
        reportBitToggle(report, WARN, 13, "Following error.", "No following error.");
        break;
    case 3: // profile velocity
        reportBitToggle(report, NONE, 12, "Speed is equal to 0.", "Speed is not equal to 0.");
        reportBitToggle(report, WARN, 13, "Maximum slippage reached.", "Maximum slippage not reached.");
        break;
    case 7: // linear interpolation
        reportBitToggle(report, INFO, 12, "Interpolated position mode active.", "Interpolated position mode inactive.");
        // 13: reserved
        break;
    }

    reportBitToggle(report, INFO, 14, "Last event set has occurred.",
            "No event set or the programmed event has not occurred yet.");
    reportBitToggle(report, INFO, 15, "Axis on. Power stage is enabled. Motor control is performed.",
            "Axis off. Power stage is disabled. Motor control is not performed.");

    can->driveStatus()->update(statusword);
}

// -----------------------------------------------------------------------------

void TechnosoftIposBase::interpretModesOfOperation(std::int8_t modesOfOperation)
{
    switch (modesOfOperation)
    {
    case -5:
        yCIInfo(IPOS, id()) << "iPOS specific: External Reference Torque Mode";
        break;
    case -4:
        yCIInfo(IPOS, id()) << "iPOS specific: External Reference Speed Mode";
        break;
    case -3:
        yCIInfo(IPOS, id()) << "iPOS specific: External Reference Position Mode";
        break;
    case -2:
        yCIInfo(IPOS, id()) << "iPOS specific: Electronic Camming Position Mode";
        break;
    case -1:
        yCIInfo(IPOS, id()) << "iPOS specific: Electronic Gearing Position Mode";
        break;
    case 1:
        yCIInfo(IPOS, id()) << "Profile Position Mode";
        break;
    case 3:
        yCIInfo(IPOS, id()) << "Profile Velocity Mode";
        break;
    case 6:
        yCIInfo(IPOS, id()) << "Homing Mode";
        break;
    case 7:
        yCIInfo(IPOS, id()) << "Interpolated Position Mode";
        break;
    case 8:
        yCIInfo(IPOS, id()) << "Cyclic Synchronous Position Mode";
        break;
    default:
        yCIWarning(IPOS, id()) << "No mode set";
        break;
    }
}

// -----------------------------------------------------------------------------

void TechnosoftIposBase::handleTpdo1(std::uint16_t statusword, std::uint16_t msr, std::int8_t modesOfOperation)
{
    interpretModesOfOperation(modesOfOperation); // statusword callback depends on this
    interpretStatusword(statusword);
    interpretMsr(msr);
}

// -----------------------------------------------------------------------------

void TechnosoftIposBase::handleTpdo2(std::uint16_t mer, std::uint16_t der)
{
    interpretMer(mer);
    interpretDer(der);
}

// -----------------------------------------------------------------------------

void TechnosoftIposBase::handleTpdo3(std::int32_t position, std::int16_t current)
{
    vars.lastEncoderRead->update(position);
    vars.lastCurrentRead = current;
}

// -----------------------------------------------------------------------------

void TechnosoftIposBase::handleEmcy(EmcyConsumer::code_t code, std::uint8_t reg, const std::uint8_t * msef)
{
    switch (const auto & [emcyCode, emcyMessage] = code; emcyCode)
    {
    case 0x7300:
    {
        std::uint16_t der2;
        std::memcpy(&der2, msef, 2);

        if (der2 != 0) // available only in F514x
        {
            interpretDer2(der2);
        }

        break;
    }
    case 0x7500:
    {
        std::uint16_t cer;
        std::memcpy(&cer, msef, 2);
        interpretCer(cer);
        break;
    }
    case 0xFF01:
    {
        std::uint16_t ipStatus;
        std::memcpy(&ipStatus, msef, 2);
        interpretIpStatus(ipStatus);
        break;
    }
    default:
        yCIWarning(IPOS, id()) << emcyMessage;
        break;
    }
}

// -----------------------------------------------------------------------------

void TechnosoftIposBase::handleNmt(NmtState state)
{
    vars.lastHeartbeat = yarp::os::Time::now();
    std::uint8_t nmtState = static_cast<std::uint8_t>(state);

    // always report boot-up
    if (state != NmtState::BOOTUP && vars.lastNmtState == nmtState)
    {
        return;
    }

    std::string s;

    switch (state)
    {
    case NmtState::BOOTUP:
        s = "boot-up";
        break;
    case NmtState::STOPPED:
        s = "stopped";
        break;
    case NmtState::OPERATIONAL:
        s = "operational";
        break;
    case NmtState::PRE_OPERATIONAL:
        s = "pre-operational";
        break;
    default:
        yCIWarning(IPOS, id()) << "Unhandled state:" << static_cast<std::uint8_t>(state);
        return;
    }

    yCIInfo(IPOS, id()) << "Heartbeat:" << s;

    vars.lastNmtState = nmtState;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::monitorWorker(const yarp::os::YarpTimerEvent & event)
{
    bool isConfigured = vars.actualControlMode != VOCAB_CM_NOT_CONFIGURED;
    double elapsed = event.currentReal - vars.lastHeartbeat;

    if (vars.heartbeatPeriod != 0.0 && isConfigured && elapsed > event.lastDuration)
    {
        yCIError(IPOS, id()) << "Last heartbeat response was" << elapsed << "seconds ago";
        vars.actualControlMode = VOCAB_CM_NOT_CONFIGURED;
        can->nmt()->issueServiceCommand(NmtService::RESET_NODE);
        can->driveStatus()->reset();
        vars.reset();
        reset();
    }
    else if (!isConfigured && elapsed < event.lastDuration && vars.lastNmtState == 0) // boot-up event
    {
        if (!initialize())
        {
            yCIError(IPOS, id()) << "Unable to initialize CAN comms";
            can->nmt()->issueServiceCommand(NmtService::RESET_NODE);
        }
    }

    return true;
}

// -----------------------------------------------------------------------------
