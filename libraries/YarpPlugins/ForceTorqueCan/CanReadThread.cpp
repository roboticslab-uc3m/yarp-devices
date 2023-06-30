// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanReadThread.hpp"

#include <cstdint>

#include <iomanip>
#include <sstream>
#include <string>

#include <yarp/os/Log.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto RX_DELAY = 0.001; // [s] arbitrary, but small enough to allow the NWS to get fresh data
constexpr auto BUFFER_SIZE = 500;

// -----------------------------------------------------------------------------

namespace
{
    // from CanUtils.cpp
    std::string msgToStr(std::size_t len, const std::uint8_t * data)
    {
        std::stringstream tmp;

        for (std::size_t i = 0; i < len; i++)
        {
            tmp << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(data[i]);

            if (i != len - 1)
            {
                tmp << " ";
            }
        }

        return tmp.str();
    }
}

// -----------------------------------------------------------------------------

bool CanReadThread::threadInit()
{
    canBuffer = iCanBufferFactory->createBuffer(BUFFER_SIZE);
    return true;
}

// -----------------------------------------------------------------------------

void CanReadThread::threadRelease()
{
    iCanBufferFactory->destroyBuffer(canBuffer);
}

// -----------------------------------------------------------------------------

void CanReadThread::run()
{
    unsigned int read, i;
    std::uint64_t data;

    while (!isStopping())
    {
        yarp::os::SystemClock::delaySystem(RX_DELAY);

        if (!iCanBus->canRead(canBuffer, BUFFER_SIZE, &read) || read == 0)
        {
            continue;
        }

        for (i = 0; i < read; i++)
        {
            const auto & msg = canBuffer[i];

            if (msg.getId() == 0x001)
            {
                yCDebug(FTC, "Received from id %d: %s", msg.getId(), msgToStr(msg.getLen(), msg.getData()).c_str());

                std::memcpy(&data, msg.getData(), sizeof(data));
                interpretMessage(data & 0x000000000005FFFF);
                interpretMessage((data & 0x000000FFFFF00000) >> 20);
                interpretMessage((data & 0x0FFFFF0000000000) >> 40);
            }
        }
    }
}

// -----------------------------------------------------------------------------

void CanReadThread::interpretMessage(std::uint32_t msg)
{
    std::uint8_t channel = (msg & 0x000F0000) >> 16;
    std::uint16_t data = (msg & 0x0000FFFF);

    std::lock_guard lock(msgMutex);

    switch (channel)
    {
    case 1:
        fx = static_cast<std::int16_t>(data) * 1150.0 / 16384;
        break;
    case 2:
        fy = static_cast<std::int16_t>(data) * 1110.0 / 16384;
        break;
    case 3:
        fz = static_cast<std::int16_t>(data) * 1850.0 / 16384;
        break;
    case 4:
        mx = static_cast<std::int16_t>(data) * 56.0 / 16384;
        break;
    case 5:
        my = static_cast<std::int16_t>(data) * 52.0 / 16384;
        break;
    case 6:
        mz = static_cast<std::int16_t>(data) * 61.0 / 16384;
        break;
    default:
        break;
    }
}

// -----------------------------------------------------------------------------

yarp::sig::Vector CanReadThread::getMeasurements() const
{
    std::lock_guard lock(msgMutex);
    return {fx, fy, fz, mx, my, mz};
}

// -----------------------------------------------------------------------------
