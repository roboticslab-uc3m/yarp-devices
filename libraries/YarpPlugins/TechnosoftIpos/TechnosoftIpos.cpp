// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cstring>

#include <bitset>

#include <ColorDebug.h>

using namespace roboticslab;

namespace
{
    void interpretPtEmcy(std::uint16_t status, int canId, const LinearInterpolationBuffer * buffer)
    {
        CD_INFO("Interpolated position mode status. canId: %d.\n", canId);
        std::bitset<16> bits(status);

        if (bits.test(15))
        {
            CD_INFO("\t* buffer is empty.\n");

            if (buffer->getType() == "pvt")
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
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::statuswordCb(std::uint16_t statusword)
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
}

// -----------------------------------------------------------------------------

void TechnosoftIpos::modesOfOperationCb(std::int8_t modesOfOperation)
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

void TechnosoftIpos::emcyCb(EmcyConsumer::code_t code, std::uint8_t reg, const std::uint8_t * msef)
{
    if (code.first == 0xFF01)
    {
        std::uint16_t status;
        std::memcpy(&status, msef + 3, 2);
        interpretPtEmcy(status, can->getId(), linInterpBuffer);
    }
}

// -----------------------------------------------------------------------------
