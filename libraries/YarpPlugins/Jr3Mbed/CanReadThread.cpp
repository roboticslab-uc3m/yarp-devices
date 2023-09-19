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

            if ((msg.getId() & 0x07F) == 0x001)
            {
                yCDebug(JR3M, "Received from id %d: %s", msg.getId(), msgToStr(msg.getLen(), msg.getData()).c_str());

                std::memcpy(&data, msg.getData(), sizeof(data));

                interpretMessage((msg.getId() & 0x780) >> 7,
                                 data & 0x000000000000FFFF,
                                 (data & 0x00000000FFFF0000) >> 16,
                                 (data & 0x0000FFFF00000000) >> 32);
            }
        }
    }
}

// -----------------------------------------------------------------------------

void CanReadThread::interpretMessage(std::uint8_t op, std::int16_t val1, std::int16_t val2, std::int16_t val3)
{
    std::lock_guard lock(msgMutex);

    switch (op)
    {
    case 7:
        fx = val1 * 1150.0 / 16384;
        fy = val2 * 1110.0 / 16384;
        fz = val3 * 1850.0 / 16384;
        break;
    case 8:
        mx = val1 * 56.0 / 16384;
        my = val2 * 52.0 / 16384;
        mz = val3 * 61.0 / 16384;
        break;
    default:
        yCWarning(JR3M, "Unknown operation %d", op);
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
