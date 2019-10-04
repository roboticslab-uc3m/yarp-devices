// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>

#include <bitset>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretMsr(std::uint16_t msr)
{
    std::bitset<16> bits(msr);

    vars.reportBitToggle(bits, vars.msr, 0, "Drive/motor initialization performed.");
    vars.reportBitToggle(bits, vars.msr, 1, "Position trigger 1 reached.");
    vars.reportBitToggle(bits, vars.msr, 2, "Position trigger 2 reached.");
    vars.reportBitToggle(bits, vars.msr, 3, "Position trigger 3 reached.");
    vars.reportBitToggle(bits, vars.msr, 4, "Position trigger 4 reached.");
    vars.reportBitToggle(bits, vars.msr, 5, "AUTORUN mode enabled.");
    vars.reportBitToggle(bits, vars.msr, 6, "Limit switch positive event / interrupt triggered.");
    vars.reportBitToggle(bits, vars.msr, 7, "Limit switch negative event / interrupt triggered.");
    vars.reportBitToggle(bits, vars.msr, 8, "Capture event/interrupt triggered.");
    vars.reportBitToggle(bits, vars.msr, 9, "Target command reached.");
    vars.reportBitToggle(bits, vars.msr, 10, "Motor I2t protection warning level reached.");
    vars.reportBitToggle(bits, vars.msr, 11, "Drive I2t protection warning level reached.");
    vars.reportBitToggle(bits, vars.msr, 12, "Gear ratio in electronic gearing mode reached.");
    // 13 (29): reserved
    vars.reportBitToggle(bits, vars.msr, 14, "Reference position in absolute electronic camming mode reached.");
    vars.reportBitToggle(bits, vars.msr, 15, "Drive/motor in fault status.");

    vars.msr = msr;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretMer(std::uint16_t mer)
{
    std::bitset<16> bits(mer);

    vars.reportBitToggle(bits, vars.mer, 0, "CAN error. Set when CAN controller is in error mode.");
    vars.reportBitToggle(bits, vars.mer, 1, "Short-circuit. Set when protection is triggered.");
    vars.reportBitToggle(bits, vars.mer, 2, "Invalid setup data. Set when the EEPROM stored setup data is not valid or not present.");
    vars.reportBitToggle(bits, vars.mer, 3, "Control error (position/speed error too big). Set when protection is triggered.");
    vars.reportBitToggle(bits, vars.mer, 4, "Communication error. Set when protection is triggered.");
    vars.reportBitToggle(bits, vars.mer, 5, "Motor position wraps around. Set when protection is triggered.");
    vars.reportBitToggle(bits, vars.mer, 6, "Positive limit switch active. Set when LSP input is in active state.");
    vars.reportBitToggle(bits, vars.mer, 7, "Negative limit switch active. Set when LSN input is in active state.");
    vars.reportBitToggle(bits, vars.mer, 8, "Over current. Set when protection is triggered.");
    vars.reportBitToggle(bits, vars.mer, 9, "I2t protection. Set when protection is triggered.");
    vars.reportBitToggle(bits, vars.mer, 10, "Over temperature motor. Set when protection is triggered.");
    vars.reportBitToggle(bits, vars.mer, 11, "Over temperature drive. Set when protection is triggered.");
    vars.reportBitToggle(bits, vars.mer, 12, "Over-voltage. Set when protection is triggered.");
    vars.reportBitToggle(bits, vars.mer, 13, "Under-voltage. Set when protection is triggered.");
    vars.reportBitToggle(bits, vars.mer, 14, "Command error.");
    vars.reportBitToggle(bits, vars.mer, 15, "Drive disabled due to enable or STO input. Set when enable or STO input is on disable state.");

    vars.mer = mer;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretDer(std::uint16_t der)
{
    std::bitset<16> bits(der);

    vars.reportBitToggle(bits, vars.der, 0, "The number of nested function calls exceeded the length of TML stack. Last function call was ignored.");
    vars.reportBitToggle(bits, vars.der, 1, "A RET/RETI instruction was executed while no function/ISR was active.");
    vars.reportBitToggle(bits, vars.der, 2, "A call to an inexistent homing routine was received.");
    vars.reportBitToggle(bits, vars.der, 3, "A call to an inexistent function was received.");
    vars.reportBitToggle(bits, vars.der, 4, "UPD instruction received while AXISON was executed. The UPD instruction was ignored and it must be sent again when AXISON is completed.");
    vars.reportBitToggle(bits, vars.der, 5, "Cancelable call instruction received while another cancelable function was active.");
    vars.reportBitToggle(bits, vars.der, 6, "Positive software limit switch is active.");
    vars.reportBitToggle(bits, vars.der, 7, "Negative software limit switch is active.");
    vars.reportBitToggle(bits, vars.der, 8, "S-curve parameters caused and invalid profile. UPD instruction was ignored.");
    vars.reportBitToggle(bits, vars.der, 9, "Update ignored for S-curve.");
    vars.reportBitToggle(bits, vars.der, 10, "Encoder broken wire; On a brushless motor, either the digital halls or the incremental encoder signal was interrupted.");
    vars.reportBitToggle(bits, vars.der, 11, "Start mode failed. Motionless start or pole lock minimum movement failed.");
    // 12: reserved
    vars.reportBitToggle(bits, vars.der, 13, "Self-check error. Internal memory (OTP) checksum error.");
    vars.reportBitToggle(bits, vars.der, 14, "STO or Enable circuit hardware error.");
    vars.reportBitToggle(bits, vars.der, 15, "EEPROM Locked. An attempt to write in the EEPROM will be ignored.");

    vars.der = der;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretDer2(std::uint16_t der2)
{
    std::bitset<16> bits(der2);

    vars.reportBitToggle(bits, vars.der2, 0, "BiSS data CRC error");
    vars.reportBitToggle(bits, vars.der2, 1, "BiSS data warning bit is set");
    vars.reportBitToggle(bits, vars.der2, 2, "BiSS data error bit is set");
    vars.reportBitToggle(bits, vars.der2, 3, "BiSS sensor missing. No BiSS sensor communication detected.");
    vars.reportBitToggle(bits, vars.der2, 4, "Absolute Encoder Interface (AEI) interface error.");
    vars.reportBitToggle(bits, vars.der2, 5, "Hall sensor missing. Can be either Digital or Linear analogue hall error.");
    vars.reportBitToggle(bits, vars.der2, 6, "Position wraparound.");
    // 6-15: reserved

    vars.der2 = der2;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretCer(std::uint16_t cer)
{
    std::bitset<16> bits(cer);

    vars.reportBitToggle(bits, vars.cer, 0, "RS232 reception error.");
    vars.reportBitToggle(bits, vars.cer, 1, "RS232 transmission timeout error.");
    vars.reportBitToggle(bits, vars.cer, 2, "RS232 reception timeout error.");
    vars.reportBitToggle(bits, vars.cer, 3, "CAN reception timeout error.");
    vars.reportBitToggle(bits, vars.cer, 4, "CAN reception overrun error.");
    vars.reportBitToggle(bits, vars.cer, 5, "CAN transmission overrun error.");
    vars.reportBitToggle(bits, vars.cer, 6, "CAN bus off error.");
    vars.reportBitToggle(bits, vars.cer, 7, "SPI timeout on write operation.");
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
    }

    std::bitset<16> bits(statusword);
    const std::bitset<16> & stored = can->driveStatus()->statusword();

    vars.reportBitToggle(bits, stored, 0, "Ready to switch on.");
    vars.reportBitToggle(bits, stored, 1, "Switched on.");
    vars.reportBitToggle(bits, stored, 2, "Operation enabled.");
    vars.reportBitToggle(bits, stored, 3, "Fault.");
    vars.reportBitToggle(bits, stored, 4, "Motor supply voltage is present.", "Motor supply voltage is absent");
    vars.reportBitToggle(bits, stored, 5, "Quick Stop.");
    vars.reportBitToggle(bits, stored, 6, "Switch on disabled.");
    vars.reportBitToggle(bits, stored, 7, "Warning. A TML function / homing was called, while another TML function / homing is still in execution. The last call is ignored.", "No warning.");
    vars.reportBitToggle(bits, stored, 8, "A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called.", "No TML function or homing is executed. The execution of the last called TML function or homing is completed.");
    vars.reportBitToggle(bits, stored, 9, "Remote - drive parameters may be modified via CAN and the drive will execute the command message.", "Remote – drive is in local mode and will not execute the command message.");
    vars.reportBitToggle(bits, stored, 10, "Target reached.");
    vars.reportBitToggle(bits, stored, 11, "Internal Limit Active.");

    switch (vars.actualControlMode.load())
    {
    case VOCAB_CM_POSITION:
        vars.reportBitToggle(bits, stored, 12, "Trajectory generator will not accept a new set-point.", "Trajectory generator will accept a new set-point.");
        vars.reportBitToggle(bits, stored, 13, "Following error.", "No following error.");
        break;
    case VOCAB_CM_VELOCITY:
        vars.reportBitToggle(bits, stored, 12, "Speed is equal to 0.", "Speed is not equal to 0.");
        vars.reportBitToggle(bits, stored, 13, "Maximum slippage reached.", "Maximum slippage not reached.");
        break;
    case VOCAB_CM_POSITION_DIRECT:
        vars.reportBitToggle(bits, stored, 12, "Interpolated position mode active.", "Interpolated position mode inactive.");
        // 13: reserved
        break;
    }

    vars.reportBitToggle(bits, stored, 14, "Last event set has occurred.", "No event set or the programmed event has not occurred yet.");
    vars.reportBitToggle(bits, stored, 15, "Axis on. Power stage is enabled. Motor control is performed.", "Axis off. Power stage is disabled. Motor control is not performed.");

    can->driveStatus()->update(statusword);
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretModesOfOperation(std::int8_t modesOfOperation)
{
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

    vars.controlModeObserverPtr->notify();
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretPtStatus(std::uint16_t status)
{
    CD_INFO("Interpolated position mode status. canId: %d.\n", can->getId());
    std::bitset<16> bits(status);

    if (bits.test(15))
    {
        CD_INFO("\t* buffer is empty.\n");

        if (linInterpBuffer->getType() == "pvt")
        {
            if (bits.test(11))
            {
                CD_INFO("\t* pvt maintained position on buffer empty (zero velocity).\n");
            }
            else
            {
                CD_INFO("\t* pvt performed quick stop on buffer empty (non-zero velocity).\n");
            }
        }
    }
    else
    {
        CD_INFO("\t* buffer is not empty.\n");
    }

    if (bits.test(14))
    {
        CD_INFO("\t* buffer is low.\n");
    }
    else
    {
        CD_INFO("\t* buffer is not low.\n");
    }

    if (bits.test(13))
    {
        CD_INFO("\t* buffer is full.\n");
    }
    else
    {
        CD_INFO("\t* buffer is not full.\n");
    }

    if (bits.test(12))
    {
        CD_INFO("\t* integrity counter error.\n");
    }
    else
    {
        CD_INFO("\t* no integrity counter error.\n");
    }
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
    CD_WARNING("EMCY: %s (canId %d)", code.second.c_str(), can->getId());

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
