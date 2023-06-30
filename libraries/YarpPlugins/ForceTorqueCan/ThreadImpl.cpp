// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ForceTorqueCan.hpp"

#include <cstdint>

#include <iomanip>
#include <sstream>
#include <string>

#include <yarp/os/Log.h>
#include <yarp/os/SystemClock.h>

#include "LogComponent.hpp"

using namespace roboticslab;

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

bool ForceTorqueCan::threadInit()
{
    canBuffer = iCanBufferFactory->createBuffer(bufferSize);
    return true;
}

// -----------------------------------------------------------------------------

void ForceTorqueCan::threadRelease()
{
    iCanBufferFactory->destroyBuffer(canBuffer);
}

// -----------------------------------------------------------------------------

void ForceTorqueCan::run()
{
    unsigned int read, i;

    while (!isStopping())
    {
        // arbitrary, but small enough to allow the NWS to get fresh data
        yarp::os::SystemClock::delaySystem(0.001);

        if (!iCanBus->canRead(canBuffer, bufferSize, &read) || read == 0)
        {
            continue;
        }

        for (i = 0; i < read; i++)
        {
            const auto & msg = canBuffer[i];

            // TODO: for now, just log stuff
            yCDebug(FTC, "Received from id %d: %s", msg.getId(), msgToStr(msg.getLen(), msg.getData()).c_str());
        }
    }
}

// -----------------------------------------------------------------------------
