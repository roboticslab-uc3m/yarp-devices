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
            static_cast<std::int16_t> (data & 0x000000000000FFFF),
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

    bool ret = false;

    switch (mode)
    {
    case jr3_mode::SYNC:
        ret = sendStartSyncCommand(filter);
        break;
    case jr3_mode::ASYNC:
        ret = sendStartAsyncCommand(filter, asyncPeriod);
        break;
    default:
        yCIError(JR3M, id()) << "Unknown mode:" << static_cast<int>(mode);
        return false;
    }

    ret = ret && (!shouldQueryFullScales || queryFullScales());

    // wait for the sensor data to settle, otherwise zeroing will be inaccurate
    ret = ret && (yarp::os::SystemClock::delaySystem(0.1), sendCommand("zero offsets", can_ops::ZERO_OFFS));

    return ret;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::finalize()
{
    if (monitorThread && monitorThread->isRunning())
    {
        monitorThread->stop();
    }

    return sendCommand("stop", can_ops::STOP);
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::notifyMessage(const can_message & message)
{
    switch (static_cast<can_ops>(message.id - canId))
    {
    case can_ops::BOOTUP:
    {
        yCIInfo(JR3M, id()) << "Bootup message received";
        isBooting = true;
        // can't block here, let the monitor thread call the initialization routine
        return true;
    }
    case can_ops::ACK:
        return ackStateObserver->notify(message.data, message.len);
    case can_ops::FORCES:
    {
        auto [forces, counter] = parseData(message);
        std::lock_guard lock(mtx);
        buffer = forces;
        frameCounter = counter;
        return true;
    }
    case can_ops::MOMENTS:
    {
        auto [moments, counter] = parseData(message);

        if (std::lock_guard lock(mtx); counter == frameCounter)
        {
            std::copy(buffer.cbegin(), buffer.cend(), raw.begin());
            std::copy(moments.cbegin(), moments.cend(), raw.begin() + 3);
            timestamp = yarp::os::SystemClock::nowSystem();
        }

        return true;
    }
    default:
        yCIWarning(JR3M, id(), "Unsupported operation: 0x%02x", message.id - canId);
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
