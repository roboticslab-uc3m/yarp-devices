// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>

#include <bitset>
#include <string>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    template<std::size_t N>
    bool reportBitToggle(unsigned int canId, const std::bitset<N> & actual, const std::bitset<N> & stored,
            std::size_t pos, const std::string & msgSet, const std::string & msgReset = "")
    {
        if (actual.test(pos) == stored.test(pos))
        {
            return false;
        }

        if (actual.test(pos))
        {
            CD_INFO("%s (canId: %d)\n", msgSet.c_str(), canId);
        }
        else
        {
            if (!msgReset.empty())
            {
                CD_INFO("%s (canId: %d)\n", msgReset.c_str(), canId);
            }
            else
            {
                CD_INFO("Bit reset: %s (canId: %d)\n", msgSet.c_str(), canId);
            }
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
    std::bitset<16> bits(msr);
    std::bitset<16> stored(vars.msr);
    unsigned int id = can->getId();

    reportBitToggle(id, bits, stored, 0, "Drive/motor initialization performed.");
    //reportBitToggle(id, bits, stored, 1, "Position trigger 1 reached."); // too verbose in position profile mode
    //reportBitToggle(id, bits, stored, 2, "Position trigger 2 reached.");
    //reportBitToggle(id, bits, stored, 3, "Position trigger 3 reached.");
    //reportBitToggle(id, bits, stored, 4, "Position trigger 4 reached.");
    reportBitToggle(id, bits, stored, 5, "AUTORUN mode enabled.");
    reportBitToggle(id, bits, stored, 6, "Limit switch positive event / interrupt triggered.");
    reportBitToggle(id, bits, stored, 7, "Limit switch negative event / interrupt triggered.");
    reportBitToggle(id, bits, stored, 8, "Capture event/interrupt triggered.");
    reportBitToggle(id, bits, stored, 9, "Target command reached.");
    reportBitToggle(id, bits, stored, 10, "Motor I2t protection warning level reached.");
    reportBitToggle(id, bits, stored, 11, "Drive I2t protection warning level reached.");
    reportBitToggle(id, bits, stored, 12, "Gear ratio in electronic gearing mode reached.");
    // 13 (29): reserved
    reportBitToggle(id, bits, stored, 14, "Reference position in absolute electronic camming mode reached.");
    reportBitToggle(id, bits, stored, 15, "Drive/motor in fault status.");

    vars.msr = msr;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretMer(std::uint16_t mer)
{
    std::bitset<16> bits(mer);
    std::bitset<16> stored(vars.mer);
    unsigned int id = can->getId();

    reportBitToggle(id, bits, stored, 0, "CAN error. Set when CAN controller is in error mode.");
    reportBitToggle(id, bits, stored, 1, "Short-circuit. Set when protection is triggered.");
    reportBitToggle(id, bits, stored, 2, "Invalid setup data. Set when the EEPROM stored setup data is not valid or not present.");
    reportBitToggle(id, bits, stored, 3, "Control error (position/speed error too big). Set when protection is triggered.");
    reportBitToggle(id, bits, stored, 4, "Communication error. Set when protection is triggered.");
    reportBitToggle(id, bits, stored, 5, "Motor position wraps around. Set when protection is triggered.");
    reportBitToggle(id, bits, stored, 6, "Positive limit switch active. Set when LSP input is in active state.");
    reportBitToggle(id, bits, stored, 7, "Negative limit switch active. Set when LSN input is in active state.");
    reportBitToggle(id, bits, stored, 8, "Over current. Set when protection is triggered.");
    reportBitToggle(id, bits, stored, 9, "I2t protection. Set when protection is triggered.");
    reportBitToggle(id, bits, stored, 10, "Over temperature motor. Set when protection is triggered.");
    reportBitToggle(id, bits, stored, 11, "Over temperature drive. Set when protection is triggered.");
    reportBitToggle(id, bits, stored, 12, "Over-voltage. Set when protection is triggered.");
    reportBitToggle(id, bits, stored, 13, "Under-voltage. Set when protection is triggered.");
    reportBitToggle(id, bits, stored, 14, "Command error.");
    reportBitToggle(id, bits, stored, 15, "Drive disabled due to enable or STO input. Set when enable or STO input is on disable state.");

    vars.mer = mer;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretDer(std::uint16_t der)
{
    std::bitset<16> bits(der);
    std::bitset<16> stored(vars.der);
    unsigned int id = can->getId();

    reportBitToggle(id, bits, stored, 0, "The number of nested function calls exceeded the length of TML stack. Last function call was ignored.");
    reportBitToggle(id, bits, stored, 1, "A RET/RETI instruction was executed while no function/ISR was active.");
    reportBitToggle(id, bits, stored, 2, "A call to an inexistent homing routine was received.");
    reportBitToggle(id, bits, stored, 3, "A call to an inexistent function was received.");
    reportBitToggle(id, bits, stored, 4, "UPD instruction received while AXISON was executed. The UPD instruction was ignored and it must be sent again when AXISON is completed.");
    reportBitToggle(id, bits, stored, 5, "Cancelable call instruction received while another cancelable function was active.");
    reportBitToggle(id, bits, stored, 6, "Positive software limit switch is active.");
    reportBitToggle(id, bits, stored, 7, "Negative software limit switch is active.");
    reportBitToggle(id, bits, stored, 8, "S-curve parameters caused and invalid profile. UPD instruction was ignored.");
    reportBitToggle(id, bits, stored, 9, "Update ignored for S-curve.");
    reportBitToggle(id, bits, stored, 10, "Encoder broken wire; On a brushless motor, either the digital halls or the incremental encoder signal was interrupted.");
    reportBitToggle(id, bits, stored, 11, "Start mode failed. Motionless start or pole lock minimum movement failed.");
    // 12: reserved
    reportBitToggle(id, bits, stored, 13, "Self-check error. Internal memory (OTP) checksum error.");
    reportBitToggle(id, bits, stored, 14, "STO or Enable circuit hardware error.");
    reportBitToggle(id, bits, stored, 15, "EEPROM Locked. An attempt to write in the EEPROM will be ignored.");

    vars.der = der;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretDer2(std::uint16_t der2)
{
    std::bitset<16> bits(der2);
    std::bitset<16> stored(vars.der2);
    unsigned int id = can->getId();

    reportBitToggle(id, bits, stored, 0, "BiSS data CRC error");
    reportBitToggle(id, bits, stored, 1, "BiSS data warning bit is set");
    reportBitToggle(id, bits, stored, 2, "BiSS data error bit is set");
    reportBitToggle(id, bits, stored, 3, "BiSS sensor missing. No BiSS sensor communication detected.");
    reportBitToggle(id, bits, stored, 4, "Absolute Encoder Interface (AEI) interface error.");
    reportBitToggle(id, bits, stored, 5, "Hall sensor missing. Can be either Digital or Linear analogue hall error.");
    reportBitToggle(id, bits, stored, 6, "Position wraparound.");
    // 6-15: reserved

    vars.der2 = der2;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretCer(std::uint16_t cer)
{
    std::bitset<16> bits(cer);
    std::bitset<16> stored(vars.cer);
    unsigned int id = can->getId();

    reportBitToggle(id, bits, stored, 0, "RS232 reception error.");
    reportBitToggle(id, bits, stored, 1, "RS232 transmission timeout error.");
    reportBitToggle(id, bits, stored, 2, "RS232 reception timeout error.");
    reportBitToggle(id, bits, stored, 3, "CAN reception timeout error.");
    reportBitToggle(id, bits, stored, 4, "CAN reception overrun error.");
    reportBitToggle(id, bits, stored, 5, "CAN transmission overrun error.");
    reportBitToggle(id, bits, stored, 6, "CAN bus off error.");
    reportBitToggle(id, bits, stored, 7, "SPI timeout on write operation.");
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

    std::bitset<16> bits(statusword);
    const std::bitset<16> & stored = can->driveStatus()->statusword();
    unsigned int id = can->getId();

    reportBitToggle(id, bits, stored, 0, "Ready to switch on.");
    reportBitToggle(id, bits, stored, 1, "Switched on.");
    reportBitToggle(id, bits, stored, 2, "Operation enabled.");
    reportBitToggle(id, bits, stored, 3, "Fault.");
    reportBitToggle(id, bits, stored, 4, "Motor supply voltage is present.", "Motor supply voltage is absent");
    reportBitToggle(id, bits, stored, 5, "Quick Stop.");
    reportBitToggle(id, bits, stored, 6, "Switch on disabled.");
    reportBitToggle(id, bits, stored, 7, "Warning. A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored.", "No warning.");
    reportBitToggle(id, bits, stored, 8, "A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called.", "No TML function or homing is executed. The execution of the last called TML function or homing is completed.");
    reportBitToggle(id, bits, stored, 9, "Remote - drive parameters may be modified via CAN and the drive will execute the command message.", "Remote – drive is in local mode and will not execute the command message.");
    reportBitToggle(id, bits, stored, 10, "Target reached.");
    reportBitToggle(id, bits, stored, 11, "Internal Limit Active.");

    switch (vars.actualControlMode.load())
    {
    case VOCAB_CM_POSITION:
        reportBitToggle(id, bits, stored, 12, "Trajectory generator will not accept a new set-point.", "Trajectory generator will accept a new set-point.");
        reportBitToggle(id, bits, stored, 13, "Following error.", "No following error.");
        break;
    case VOCAB_CM_VELOCITY:
        //reportBitToggle(id, bits, stored, 12, "Speed is equal to 0.", "Speed is not equal to 0."); // too verbose
        reportBitToggle(id, bits, stored, 13, "Maximum slippage reached.", "Maximum slippage not reached.");
        break;
    case VOCAB_CM_POSITION_DIRECT:
        reportBitToggle(id, bits, stored, 12, "Interpolated position mode active.", "Interpolated position mode inactive.");
        // 13: reserved
        break;
    }

    reportBitToggle(id, bits, stored, 14, "Last event set has occurred.", "No event set or the programmed event has not occurred yet.");
    reportBitToggle(id, bits, stored, 15, "Axis on. Power stage is enabled. Motor control is performed.", "Axis off. Power stage is disabled. Motor control is not performed.");

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
        CD_WARNING("Mode \"%d\" not specified in manual, may be in Fault or not enabled yet. canId: %d.\n", modesOfOperation, can->getId());
        vars.actualControlMode = VOCAB_CM_UNKNOWN;
        break;
    }

    vars.modesOfOperation = modesOfOperation;
    vars.controlModeObserverPtr->notify();
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretPtStatus(std::uint16_t status)
{
    std::bitset<16> bits(status);
    const std::bitset<16> & stored = vars.ptStatus;
    unsigned int id = can->getId();

    std::uint8_t ic = status & 0x007F; // integrity counter
    // 7-10: reserved
    reportBitToggle(id, bits, stored, 11, "Drive has maintained interpolated position mode after a buffer empty condition.",
            "Drive has performed a quick stop after a buffer empty condition (last velocity was non-zero).");
    reportBitToggle(id, bits, stored, 12, "No integrity counter error.", "Integrity counter error.");
    reportBitToggle(id, bits, stored, 13, "Buffer is not full.", "Buffer is full.");
    reportBitToggle(id, bits, stored, 14, "Buffer is not low.", "Buffer is low.");
    reportBitToggle(id, bits, stored, 15, "Buffer is empty.", "Buffer is not empty.");
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::handleTpdo1(std::uint16_t statusword, std::uint16_t msr, std::int8_t modesOfOperation)
{
    interpretStatusword(statusword);
    interpretMsr(msr);
    interpretModesOfOperation(modesOfOperation);
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
    CD_WARNING("EMCY: %s (canId %d)\n", code.second.c_str(), can->getId());

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
        }
        break;

    case 0x7500:
        {
            std::uint16_t cer;
            std::memcpy(&cer, msef, 2);
            interpretCer(cer);
        }
        break;

    case 0xFF01:
        {
            std::uint16_t ptStatus;
            std::memcpy(&ptStatus, msef, 2);
            interpretPtStatus(ptStatus);
        }
        break;
    }
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
