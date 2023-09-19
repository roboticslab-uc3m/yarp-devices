// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <cstdint>
#include <cstring>

#include <yarp/os/LogComponent.h>

#include "LogComponent.hpp"

using namespace roboticslab;

namespace
{
    // keep this in sync with the firmware
    enum can_ops
    {
        JR3_START_SYNC = 2, // 0x100
        JR3_START_ASYNC,    // 0x180
        JR3_STOP,           // 0x200
        JR3_ZERO_OFFS,      // 0x280
        JR3_SET_FILTER,     // 0x300
        JR3_GET_FORCES,     // 0x380
        JR3_GET_MOMENTS,    // 0x400
        JR3_ACK,            // 0x480
    };
}

// -----------------------------------------------------------------------------

unsigned int Jr3Mbed::getId()
{
    return canId;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::initialize()
{
    return true;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::finalize()
{
    return true;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::notifyMessage(const can_message & message)
{
    unsigned int op = (message.id - canId) >> 7;

    std::uint64_t data;
    std::memcpy(&data, message.data, message.len);

    switch (op)
    {
    case JR3_GET_FORCES:
    {
        std::lock_guard lock(rxMutex);
        fx = (data & 0x000000000000FFFF) * 1150.0 / 16384;
        fy = ((data & 0x00000000FFFF0000) >> 16) * 1110.0 / 16384;
        fz = ((data & 0x0000FFFF00000000) >> 32) * 1850.0 / 16384;
        break;
    }
    case JR3_GET_MOMENTS:
    {
        std::lock_guard lock(rxMutex);
        mx = (data & 0x000000000000FFFF) * 56.0 / 16384;
        my = ((data & 0x00000000FFFF0000) >> 16) * 52.0 / 16384;
        mz = ((data & 0x0000FFFF00000000) >> 32) * 61.0 / 16384;
        break;
    }
    default:
        yCWarning(JR3M, "Unknown operation %d", op);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::registerSender(ICanSenderDelegate * sender)
{
    this->sender = sender;
    return true;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::synchronize(double timestamp)
{
    return true;
}

// -----------------------------------------------------------------------------
