// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <cstdint>
#include <cstring>

#include <algorithm> // std::copy
#include <tuple>

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    std::tuple<std::array<std::int16_t, 3>, std::uint16_t> parseData(const can_message & message)
    {
        std::uint64_t data;
        std::memcpy(&data, message.data, message.len);

        return {{
            static_cast<std::int16_t>(data & 0x000000000000FFFF),
            static_cast<std::int16_t>((data & 0x00000000FFFF0000) >> 16),
            static_cast<std::int16_t>((data & 0x0000FFFF00000000) >> 32)
        }, static_cast<std::uint16_t>((data & 0xFFFF000000000000) >> 48)};
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
    if (!ping())
    {
        return false;
    }

    switch (mode)
    {
    case jr3_mode::SYNC:
        return sendStartSyncCommand(filter);
    case jr3_mode::ASYNC:
        return sendStartAsyncCommand(filter, asyncPeriod);
    default:
        yCIError(JR3M, id()) << "Unknown mode:" << static_cast<int>(mode);
        return false;
    }
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::finalize()
{
    return sendCommand("stop", can_ops::STOP);
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::notifyMessage(const can_message & message)
{
    const auto op = static_cast<can_ops>((message.id - canId) >> 7);

    switch (op)
    {
    case can_ops::BOOTUP:
    {
        yCIInfo(JR3M, id()) << "Bootup message received";
        isBooting = true;
        // can't block here, let the monitor thread call the initialization routine
        return true;
    }
    case can_ops::ACK:
        return message.len == 1 && ackStateObserver->notify(message.data[0]);
    case can_ops::FORCES:
    {
        auto [forces, counter] = parseData(message);
        std::lock_guard lock(mtx);
        buffer = forces;
        integrityCounter = counter;
        return true;
    }
    case can_ops::MOMENTS:
    {
        auto [moments, counter] = parseData(message);

        if (std::lock_guard lock(mtx); counter == integrityCounter)
        {
            std::copy(buffer.cbegin(), buffer.cend(), raw.begin());
            std::copy(moments.cbegin(), moments.cend(), raw.begin() + 3);
            timestamp = yarp::os::SystemClock::nowSystem();
        }

        return true;
    }
    default:
        yCIWarning(JR3M, id()) << "Unsupported operation:" << static_cast<unsigned int>(op);
        return false;
    }
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
