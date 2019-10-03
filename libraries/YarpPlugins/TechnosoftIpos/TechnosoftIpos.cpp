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

    vars.reportBitToggle("Drive/motor initialization performed.", bits, vars.msr, 0);
    vars.reportBitToggle("Position trigger 1 reached.", bits, vars.msr, 1);
    vars.reportBitToggle("Position trigger 2 reached.", bits, vars.msr, 2);
    vars.reportBitToggle("Position trigger 3 reached.", bits, vars.msr, 3);
    vars.reportBitToggle("Position trigger 4 reached.", bits, vars.msr, 4);
    vars.reportBitToggle("AUTORUN mode enabled.", bits, vars.msr, 5);
    vars.reportBitToggle("Limit switch positive event / interrupt triggered.", bits, vars.msr, 6);
    vars.reportBitToggle("Limit switch negative event / interrupt triggered.", bits, vars.msr, 7);
    vars.reportBitToggle("Capture event/interrupt triggered.", bits, vars.msr, 8);
    vars.reportBitToggle("Target command reached.", bits, vars.msr, 9);
    vars.reportBitToggle("Motor I2t protection warning level reached.", bits, vars.msr, 10);
    vars.reportBitToggle("Drive I2t protection warning level reached.", bits, vars.msr, 11);
    vars.reportBitToggle("Gear ratio in electronic gearing mode reached.", bits, vars.msr, 12);
    // 13 (29): reserved
    vars.reportBitToggle("Reference position in absolute electronic camming mode reached.", bits, vars.msr, 14);
    vars.reportBitToggle("Drive/motor in fault status.", bits, vars.msr, 15);

    vars.msr = msr;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretMer(std::uint16_t mer)
{
    std::bitset<16> bits(mer);

    vars.reportBitToggle("CAN error. Set when CAN controller is in error mode.", bits, vars.mer, 0);
    vars.reportBitToggle("Short-circuit. Set when protection is triggered.", bits, vars.mer, 1);
    vars.reportBitToggle("Invalid setup data. Set when the EEPROM stored setup data is not valid or not present.", bits, vars.mer, 2);
    vars.reportBitToggle("Control error (position/speed error too big). Set when protection is triggered.", bits, vars.mer, 3);
    vars.reportBitToggle("Communication error. Set when protection is triggered.", bits, vars.mer, 4);
    vars.reportBitToggle("Motor position wraps around. Set when protection is triggered.", bits, vars.mer, 5);
    vars.reportBitToggle("Positive limit switch active. Set when LSP input is in active state.", bits, vars.mer, 6);
    vars.reportBitToggle("Negative limit switch active. Set when LSN input is in active state.", bits, vars.mer, 7);
    vars.reportBitToggle("Over current. Set when protection is triggered.", bits, vars.mer, 8);
    vars.reportBitToggle("I2t protection. Set when protection is triggered.", bits, vars.mer, 9);
    vars.reportBitToggle("Over temperature motor. Set when protection is triggered.", bits, vars.mer, 10);
    vars.reportBitToggle("Over temperature drive. Set when protection is triggered.", bits, vars.mer, 11);
    vars.reportBitToggle("Over-voltage. Set when protection is triggered.", bits, vars.mer, 12);
    vars.reportBitToggle("Under-voltage. Set when protection is triggered.", bits, vars.mer, 13);
    vars.reportBitToggle("Command error, see DER2.", bits, vars.mer, 14);
    vars.reportBitToggle("Drive disabled due to enable or STO input. Set when enable or STO input is on disable state.", bits, vars.mer, 15);

    vars.mer = mer;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretDer(std::uint16_t der)
{
    std::bitset<16> bits(der);

    vars.reportBitToggle("The number of nested function calls exceeded the length of TML stack. Last function call was ignored.", bits, vars.der, 0);
    vars.reportBitToggle("A RET/RETI instruction was executed while no function/ISR was active.", bits, vars.der, 1);
    vars.reportBitToggle("A call to an inexistent homing routine was received.", bits, vars.der, 2);
    vars.reportBitToggle("A call to an inexistent function was received.", bits, vars.der, 3);
    vars.reportBitToggle("UPD instruction received while AXISON was executed. The UPD instruction was ingnored and it must be sent again when AXISON is completed.", bits, vars.der, 4);
    vars.reportBitToggle("Cancelable call instruction received while another cancelable function was active.", bits, vars.der, 5);
    vars.reportBitToggle("Positive software limit switch is active.", bits, vars.der, 6);
    vars.reportBitToggle("Negative software limit switch is active.", bits, vars.der, 7);
    vars.reportBitToggle("S-curve parameters caused and invalid profile. UPD instruction was ignored.", bits, vars.der, 8);
    vars.reportBitToggle("Update ignored for S-curve.", bits, vars.der, 9);
    vars.reportBitToggle("Encoder broken wire. On a brushless motor, either the digital halls or the incremental encoder signal was interrupted.", bits, vars.der, 10);
    vars.reportBitToggle("Start mode failed. Motionless start or pole lock minimum movement failed.", bits, vars.der, 11);
    // 12: reserved
    vars.reportBitToggle("Self-check error. Internal memory (OTP) checksum error.", bits, vars.der, 13);
    vars.reportBitToggle("STO or Enable circuit hardware error.", bits, vars.der, 14);
    vars.reportBitToggle("EEPROM Locked. An attempt to write in the EEPROM will be ignored.", bits, vars.der, 15);

    vars.der = der;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretDer2(std::uint16_t der2)
{
    std::bitset<16> bits(der2);

    vars.reportBitToggle("BiSS data CRC error", bits, vars.der2, 0);
    vars.reportBitToggle("BiSS data warning bit is set", bits, vars.der2, 1);
    vars.reportBitToggle("BiSS data error bit is set", bits, vars.der2, 2);
    vars.reportBitToggle("BiSS sensor missing. No BiSS sensor communication detected.", bits, vars.der2, 3);
    vars.reportBitToggle("Absolute Encoder Interface (AEI) interface error.", bits, vars.der2, 4);
    vars.reportBitToggle("Hall sensor missing. Can be either Digital or Linear analogue hall error.", bits, vars.der2, 5);
    vars.reportBitToggle("Position wraparound.", bits, vars.der2, 6);
    // 6-15: reserved

    vars.der2 = der2;
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::interpretCer(std::uint16_t cer)
{
    std::bitset<16> bits(cer);

    vars.reportBitToggle("RS232 reception error.", bits, vars.cer, 0);
    vars.reportBitToggle("RS232 transmission timeout error.", bits, vars.cer, 1);
    vars.reportBitToggle("RS232 reception timeout error.", bits, vars.cer, 2);
    vars.reportBitToggle("CAN reception timeout error.", bits, vars.cer, 3);
    vars.reportBitToggle("CAN reception overrun error.", bits, vars.cer, 4);
    vars.reportBitToggle("CAN transmission overrun error.", bits, vars.cer, 5);
    vars.reportBitToggle("CAN bus off error.", bits, vars.cer, 6);
    vars.reportBitToggle("SPI timeout on write operation.", bits, vars.cer, 7);
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

    can->driveStatus()->update(statusword);

    std::bitset<16> bits(statusword);

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
