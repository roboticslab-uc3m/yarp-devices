// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposEmbedded.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

void TechnosoftIposEmbedded::interpretModesOfOperation(std::int8_t modesOfOperation)
{
    if (this->modesOfOperation == modesOfOperation)
    {
        return;
    }

    switch (modesOfOperation)
    {
    case -5:
        actualControlMode = requestedcontrolMode == VOCAB_CM_TORQUE ? VOCAB_CM_TORQUE : VOCAB_CM_CURRENT;
        enableSync = true;
        break;
    case 1:
        actualControlMode = VOCAB_CM_POSITION;
        enableSync = false;
        break;
    case 3:
        actualControlMode = enableCsv ? VOCAB_CM_UNKNOWN : VOCAB_CM_VELOCITY;
        enableSync = false;
        break;
    case 7:
        actualControlMode = VOCAB_CM_POSITION_DIRECT;
        enableSync = false;
        break;
    case 8:
        actualControlMode = enableCsv ? VOCAB_CM_VELOCITY : VOCAB_CM_POSITION_DIRECT;
        enableSync = true;
        break;
    default:
        actualControlMode = VOCAB_CM_UNKNOWN;
        break;
    }

    TechnosoftIposBase::interpretModesOfOperation(modesOfOperation);
}

// -----------------------------------------------------------------------------

void TechnosoftIposEmbedded::interpretIpStatus(std::uint16_t status)
{
    report_storage report{"ip", status, ipStatus};

    std::uint8_t ic = status & 0x007F; // integrity counter

    // 7-10: reserved
    reportBitToggle(report, INFO, 11, "Drive has performed a quick stop after a buffer empty condition (last velocity was non-zero).",
            "Drive has maintained interpolated position mode after a buffer empty condition.");

    auto isBufferError = reportBitToggle(report, WARN, 12, "Integrity counter error.", "No integrity counter error.");
    auto isBufferFull = reportBitToggle(report, NONE, 13, "Buffer is full.", "Buffer is not full.");
    auto isBufferLow = reportBitToggle(report, NONE, 14, "Buffer is low.", "Buffer is not low."); // also true if empty!
    auto isBufferEmpty = reportBitToggle(report, INFO, 15, "Buffer is empty.", "Buffer is not empty.");

    if (isBufferError && ipBufferEnabled)
    {
        can->driveStatus()->controlword(can->driveStatus()->controlword().set(8)); // stop drive with profile acceleration
        ipBufferEnabled = false;
    }

    if (isBufferFull && ipBufferEnabled && !ipMotionStarted)
    {
        // enable ip mode
        ipMotionStarted = can->driveStatus()->controlword(can->driveStatus()->controlword().set(4));
    }

    if (isBufferLow && ipBufferEnabled && ipMotionStarted && !ipBuffer->isQueueEmpty() && !isBufferEmpty)
    {
        // load next batch of points into the drive's buffer (unless reported empty, in which case stop motion and replenish again)
        for (auto setpoint : ipBuffer->popBatch(false))
        {
            can->rpdo3()->write(setpoint);
        }
    }

    if (isBufferEmpty && ipBufferEnabled && ipMotionStarted)
    {
        // no elements in the queue and buffer is empty, disable ip mode
        ipMotionStarted = !can->driveStatus()->controlword(can->driveStatus()->controlword().reset(4));
        ipBufferFilled = false;
    }

    ipStatus = status;
}

// -----------------------------------------------------------------------------

void TechnosoftIposEmbedded::onPositionLimitTriggered()
{
    switch (actualControlMode.load())
    {
    case VOCAB_CM_TORQUE:
    case VOCAB_CM_CURRENT:
        commandBuffer.reset(0.0); // updates reference on next SYNC
        break;
    }
}

// -----------------------------------------------------------------------------

void TechnosoftIposEmbedded::reset()
{
    TechnosoftIposBase::reset();
    ipStatus = 0;
    ipMotionStarted = ipBufferFilled = ipBufferEnabled = false;
    enableSync = false;
    targetPosition = targetVelocity = 0.0;
}

// -----------------------------------------------------------------------------
