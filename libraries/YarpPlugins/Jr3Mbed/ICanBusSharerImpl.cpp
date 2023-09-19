// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <cstdint>
#include <cstring>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    std::vector<std::int16_t> parseData(const can_message & message)
    {
        std::uint64_t data;
        std::memcpy(&data, message.data, message.len);

        return {
            static_cast<std::int16_t>(data & 0x000000000000FFFF),
            static_cast<std::int16_t>((data & 0x00000000FFFF0000) >> 16),
            static_cast<std::int16_t>((data & 0x0000FFFF00000000) >> 32)
        };
    }
}

// -----------------------------------------------------------------------------

unsigned int Jr3Mbed::getId()
{
    return canId;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::initialize()
{
    switch (mode)
    {
    case SYNC:
        return sendStartSyncCommand(filter);
    case ASYNC:
        return sendStartAsyncCommand(filter, asyncPeriod);
    default:
        yCIError(JR3M, id()) << "Unknown mode:" << static_cast<int>(mode);
        return false;
    }
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::finalize()
{
    return sendStopCommand();
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::notifyMessage(const can_message & message)
{
    unsigned int op = (message.id - canId) >> 7;

    switch (op)
    {
    case JR3_GET_FORCES:
    {
        std::lock_guard lock(rxMutex);
        rawForces = parseData(message);
        break;
    }
    case JR3_GET_MOMENTS:
    {
        std::lock_guard lock(rxMutex);
        rawMoments = parseData(message);
        break;
    }
    case JR3_ACK:
        ackStateObserver->notify();
        break;
    default:
        yCIWarning(JR3M, id(), "Unsupported operation: %d", op);
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
