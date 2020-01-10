// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>

#include <bitset>
#include <sstream>
#include <string>

#include <yarp/os/Time.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    enum report_level { INFO, WARN };

    struct report_storage
    {
        std::string reg;
        std::bitset<16> actual;
        std::bitset<16> stored;
        unsigned int canId;
    };

    bool reportBitToggle(report_storage report, report_level level, std::size_t pos, const std::string & msgSet,
            const std::string & msgReset = "")
    {
        if (report.actual.test(pos) == report.stored.test(pos))
        {
            return false;
        }

        bool isSet = false;
        std::stringstream ss;
        ss << "[" << report.reg << "] ";

        if (report.actual.test(pos))
        {
            isSet = true;
            ss << msgSet;
        }
        else
        {
            ss << (msgReset.empty() ? "Bit reset: " + msgSet : msgReset);
        }

        ss << " (canId: " << report.canId << ")";

        if (isSet && level == WARN)
        {
            CD_WARNING("%s\n", ss.str().c_str());
        }
        else
        {
            CD_INFO("%s\n", ss.str().c_str());
        }

        return true;
    }
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretSupportedDriveModes(std::uint32_t data)
{
    CD_INFO("Supported drive modes (canId %d):\n", can->getId());

    std::bitset<32> bits(data);

    if (bits[0])
    {
        CD_INFO("\t* profiled position (pp)\n");
    }
    if (bits[1])
    {
        CD_INFO("\t* velocity (vl)\n");
    }
    if (bits[2])
    {
        CD_INFO("\t* profiled velocity (pv)\n");
    }
    if (bits[3])
    {
        CD_INFO("\t* profiled torque (tq)\n");
    }
    if (bits[5])
    {
        CD_INFO("\t* homing (hm)\n");
    }
    if (bits[6])
    {
        CD_INFO("\t* interpolated position (ip)\n");
    }
    if (bits[7])
    {
        CD_INFO("\t* cyclic synchronous position\n");
    }
    if (bits[8])
    {
        CD_INFO("\t* cyclic synchronous velocity\n");
    }
    if (bits[9])
    {
        CD_INFO("\t* cyclic synchronous torque\n");
    }
    if (bits[16])
    {
        CD_INFO("\t* electronic camming position (manufacturer specific)\n");
    }
    if (bits[17])
    {
        CD_INFO("\t* electronic gearing position (manufacturer specific)\n");
    }
    if (bits[18])
    {
        CD_INFO("\t* external reference position (manufacturer specific)\n");
    }
    if (bits[19])
    {
        CD_INFO("\t* external reference speed (manufacturer specific)\n");
    }
    if (bits[20])
    {
        CD_INFO("\t* external reference torque (manufacturer specific)\n");
    }
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretMsr(std::uint16_t msr)
{
    report_storage report{"MSR", msr, vars.msr, can->getId()};

    reportBitToggle(report, INFO, 0, "Drive/motor initialization performed.");
    //reportBitToggle(report, INFO, 1, "Position trigger 1 reached."); // too verbose in position profile mode
    //reportBitToggle(report, INFO, 2, "Position trigger 2 reached.");
    //reportBitToggle(report, INFO, 3, "Position trigger 3 reached.");
    //reportBitToggle(report, INFO, 4, "Position trigger 4 reached.");
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

void TechnosoftIpos::interpretMer(std::uint16_t mer)
{
    report_storage report{"MER", mer, vars.mer, can->getId()};

    reportBitToggle(report, WARN, 0, "CAN error. Set when CAN controller is in error mode.");
    reportBitToggle(report, WARN, 1, "Short-circuit. Set when protection is triggered.");
    reportBitToggle(report, WARN, 2, "Invalid setup data. Set when the EEPROM stored setup data is not valid or not present.");
    reportBitToggle(report, WARN, 3, "Control error (position/speed error too big). Set when protection is triggered.");
    reportBitToggle(report, WARN, 4, "Communication error. Set when protection is triggered.");
    reportBitToggle(report, WARN, 5, "Motor position wraps around. Set when protection is triggered.");
    reportBitToggle(report, WARN, 6, "Positive limit switch active. Set when LSP input is in active state.");
    reportBitToggle(report, WARN, 7, "Negative limit switch active. Set when LSN input is in active state.");
    reportBitToggle(report, WARN, 8, "Over current. Set when protection is triggered.");
    reportBitToggle(report, WARN, 9, "I2t protection. Set when protection is triggered.");
    reportBitToggle(report, WARN, 10, "Over temperature motor. Set when protection is triggered.");
    reportBitToggle(report, WARN, 11, "Over temperature drive. Set when protection is triggered.");
    reportBitToggle(report, WARN, 12, "Over-voltage. Set when protection is triggered.");
    reportBitToggle(report, WARN, 13, "Under-voltage. Set when protection is triggered.");
    reportBitToggle(report, WARN, 14, "Command error.");
    reportBitToggle(report, WARN, 15, "Drive disabled due to enable or STO input. Set when enable or STO input is on disable state.");

    vars.mer = mer;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretDer(std::uint16_t der)
{
    report_storage report{"DER", der, vars.der, can->getId()};

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

void TechnosoftIpos::interpretDer2(std::uint16_t der2)
{
    report_storage report{"DER2", der2, vars.der2, can->getId()};

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

void TechnosoftIpos::interpretCer(std::uint16_t cer)
{
    report_storage report{"CER", cer, vars.cer, can->getId()};

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

void TechnosoftIpos::interpretStatusword(std::uint16_t statusword)
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

    report_storage report{"status", statusword, can->driveStatus()->statusword(), can->getId()};

    reportBitToggle(report, INFO, 0, "Ready to switch on.");
    reportBitToggle(report, INFO, 1, "Switched on.");
    reportBitToggle(report, INFO, 2, "Operation enabled.");
    reportBitToggle(report, INFO, 3, "Fault.");
    reportBitToggle(report, INFO, 4, "Motor supply voltage is present.", "Motor supply voltage is absent");
    reportBitToggle(report, INFO, 5, "Quick Stop.");
    reportBitToggle(report, INFO, 6, "Switch on disabled.");
    reportBitToggle(report, WARN, 7, "A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored.", "No warning.");
    reportBitToggle(report, INFO, 8, "A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called.", "No TML function or homing is executed. The execution of the last called TML function or homing is completed.");
    reportBitToggle(report, INFO, 9, "Remote - drive parameters may be modified via CAN and the drive will execute the command message.", "Remote – drive is in local mode and will not execute the command message.");
    reportBitToggle(report, INFO, 10, "Target reached.");
    reportBitToggle(report, INFO, 11, "Internal Limit Active.");

    switch (vars.actualControlMode.load())
    {
    case VOCAB_CM_POSITION:
        reportBitToggle(report, INFO, 12, "Trajectory generator will not accept a new set-point.", "Trajectory generator will accept a new set-point.");
        reportBitToggle(report, WARN, 13, "Following error.", "No following error.");
        break;
    case VOCAB_CM_VELOCITY:
        //reportBitToggle(report, INFO, 12, "Speed is equal to 0.", "Speed is not equal to 0."); // too verbose
        reportBitToggle(report, WARN, 13, "Maximum slippage reached.", "Maximum slippage not reached.");
        break;
    case VOCAB_CM_POSITION_DIRECT:
        reportBitToggle(report, INFO, 12, "Interpolated position mode active.", "Interpolated position mode inactive.");
        // 13: reserved
        break;
    }

    reportBitToggle(report, INFO, 14, "Last event set has occurred.", "No event set or the programmed event has not occurred yet.");
    reportBitToggle(report, INFO, 15, "Axis on. Power stage is enabled. Motor control is performed.", "Axis off. Power stage is disabled. Motor control is not performed.");

    can->driveStatus()->update(statusword);
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretModesOfOperation(std::int8_t modesOfOperation)
{
    if (vars.modesOfOperation == modesOfOperation)
    {
        return;
    }

    switch (modesOfOperation)
    {
    // handled
    case -5:
        CD_INFO("iPOS specific: External Reference Torque Mode. canId: %d.\n", can->getId());
        vars.actualControlMode = vars.requestedcontrolMode == VOCAB_CM_TORQUE ? VOCAB_CM_TORQUE : VOCAB_CM_CURRENT;
        break;
    case 1:
        CD_INFO("Profile Position Mode. canId: %d.\n", can->getId());
        vars.actualControlMode = VOCAB_CM_POSITION;
        break;
    case 3:
        CD_INFO("Profile Velocity Mode. canId: %d.\n", can->getId());
        vars.actualControlMode = VOCAB_CM_VELOCITY;
        break;
    case 7:
        CD_INFO("Interpolated Position Mode. canId: %d.\n", can->getId());
        vars.actualControlMode = VOCAB_CM_POSITION_DIRECT;
        break;
    // unhandled
    case -4:
        CD_INFO("iPOS specific: External Reference Speed Mode. canId: %d.\n", can->getId());
        vars.actualControlMode = VOCAB_CM_UNKNOWN;
        break;
    case -3:
        CD_INFO("iPOS specific: External Reference Position Mode. canId: %d.\n", can->getId());
        vars.actualControlMode = VOCAB_CM_UNKNOWN;
        break;
    case -2:
        CD_INFO("iPOS specific: Electronic Camming Position Mode. canId: %d.\n", can->getId());
        vars.actualControlMode = VOCAB_CM_UNKNOWN;
        break;
    case -1:
        CD_INFO("iPOS specific: Electronic Gearing Position Mode. canId: %d.\n", can->getId());
        vars.actualControlMode = VOCAB_CM_UNKNOWN;
        break;
    case 6:
        CD_INFO("Homing Mode. canId: %d.\n", can->getId());
        vars.actualControlMode = VOCAB_CM_UNKNOWN;
        break;
    case 8:
        CD_INFO("Cyclic Synchronous Position Mode. canId: %d.\n", can->getId());
        vars.actualControlMode = VOCAB_CM_UNKNOWN;
        break;
    default:
        CD_WARNING("No mode set set. canId: %d.\n", can->getId());
        vars.actualControlMode = VOCAB_CM_UNKNOWN;
        break;
    }

    vars.modesOfOperation = modesOfOperation;
    vars.controlModeObserverPtr->notify();
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretPtStatus(std::uint16_t status)
{
    report_storage report{"pt", status, vars.ptStatus, can->getId()};

    std::uint8_t ic = status & 0x007F; // integrity counter
    // 7-10: reserved
    reportBitToggle(report, WARN, 11, "Drive has performed a quick stop after a buffer empty condition (last velocity was non-zero).",
            "Drive has maintained interpolated position mode after a buffer empty condition.");
    reportBitToggle(report, WARN, 12, "Integrity counter error.", "No integrity counter error.");
    reportBitToggle(report, INFO, 13, "Buffer is full.", "Buffer is not full.");
    reportBitToggle(report, INFO, 14, "Buffer is low.", "Buffer is not low.");
    reportBitToggle(report, INFO, 15, "Buffer is empty.", "Buffer is not empty.");

    vars.ptStatus = status;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::handleTpdo1(std::uint16_t statusword, std::uint16_t msr, std::int8_t modesOfOperation)
{
    interpretModesOfOperation(modesOfOperation); // statusword callback depends on this
    interpretStatusword(statusword);
    interpretMsr(msr);
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::handleTpdo2(std::uint16_t mer, std::uint16_t der)
{
    interpretMer(mer);
    interpretDer(der);
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::handleTpdo3(std::int32_t position, std::int16_t current)
{
    vars.lastEncoderRead.update(position);
    vars.lastCurrentRead = current;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::handleEmcy(EmcyConsumer::code_t code, std::uint8_t reg, const std::uint8_t * msef)
{
    switch (code.first)
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
        std::uint16_t ptStatus;
        std::memcpy(&ptStatus, msef, 2);
        interpretPtStatus(ptStatus);
        break;
    }
    default:
        CD_WARNING("%s (canId %d).\n", code.second.c_str(), can->getId());
        break;
    }
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::handleNmt(NmtState state)
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
        CD_WARNING("Unhandled state: %d.\n", static_cast<std::uint8_t>(state));
        return;
    }

    CD_INFO("Heartbeat: %s (canId %d).\n", s.c_str(), can->getId());

    vars.lastNmtState = nmtState;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::monitorWorker(const yarp::os::YarpTimerEvent & event)
{
    bool isConfigured = vars.actualControlMode != VOCAB_CM_NOT_CONFIGURED;
    double elapsed = event.currentReal - vars.lastHeartbeat;

    if (isConfigured && elapsed > event.lastDuration)
    {
        CD_ERROR("Last heartbeat response was %f seconds ago (canId %d).\n", elapsed, can->getId());
        vars.actualControlMode = VOCAB_CM_NOT_CONFIGURED;
    }
    else if (!isConfigured && elapsed <= event.lastDuration && vars.lastNmtState == 0) // boot-up event
    {
        if (!initialize())
        {
            CD_ERROR("Unable to initialize CAN comms (canId: %d).\n", can->getId());
            finalize();
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::quitHaltState(int mode)
{
    if (!can->driveStatus()->controlword()[8])
    {
        return true;
    }

    if (mode == VOCAB_CM_POSITION || mode == VOCAB_CM_VELOCITY)
    {
        if (!can->driveStatus()->statusword()[10])
        {
            return false;
        }
    }

    return can->driveStatus()->controlword(can->driveStatus()->controlword().reset(8));
}

// -----------------------------------------------------------------------------
