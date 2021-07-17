// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <sys/ioctl.h>

#include <cstring>
#include <cerrno>

#include <string>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusHico::canSetBaudRate(unsigned int rate)
{
    unsigned int id;

    if (!bitrateToId(rate, &id))
    {
        yCError(HICO) << "Unsupported bitrate value:" << rate;
        return false;
    }

    std::lock_guard<std::mutex> lockGuard(canBusReady);

    if (bitrateState.first)
    {
        if (bitrateState.second == id)
        {
            yCWarning(HICO) << "Bitrate already set";
            return true;
        }
        else
        {
            yCError(HICO) << "Bitrate already set to a different value:" << bitrateState.second;
            return false;
        }
    }

    if (::ioctl(fileDescriptor, IOC_SET_BITRATE, &id) == -1)
    {
        yCError(HICO) << "Could not set bitrate:" << std::strerror(errno);
        return false;
    }

    bitrateState.first = true;
    bitrateState.second = id;

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::canGetBaudRate(unsigned int * rate)
{
    unsigned int id;

    canBusReady.lock();
    int ret = ::ioctl(fileDescriptor, IOC_GET_BITRATE, &id);
    canBusReady.unlock();

    if (ret == -1)
    {
        yCError(HICO) << "Could not get bitrate:" << std::strerror(errno);
        return false;
    }

    if (!idToBitrate(id, rate))
    {
        yCError(HICO) << "Unrecognized bitrate id:" << id;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::canIdAdd(unsigned int id)
{
    if (filterConfig == FilterManager::DISABLED)
    {
        yCWarning(HICO) << "CAN filters are not enabled in this device";
        return true;
    }

    if (id > 0x7F)
    {
        yCError(HICO, "Invalid ID (%d > 0x7F)", id);
        return false;
    }

    std::lock_guard<std::mutex> lockGuard(canBusReady);

    if (filterManager->hasId(id))
    {
        yCWarning(HICO) << "Filter for ID" << id << "is already active";
        return true;
    }

    if (!filterManager->insertId(id))
    {
        yCError(HICO) << "Could not set filter:" << std::strerror(errno);
        return false;
    }

    if (!filterManager->isValid())
    {
        yCWarning(HICO) << "Hardware limit was hit, not all requested filters are enabled";
        return true;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::canIdDelete(unsigned int id)
{
    if (filterConfig == FilterManager::DISABLED)
    {
        yCWarning(HICO) << "CAN filters are not enabled in this device";
        return true;
    }

    if (id > 0x7F)
    {
        yCError(HICO, "Invalid ID (%d > 0x7F)", id);
        return false;
    }

    std::lock_guard<std::mutex> lockGuard(canBusReady);

    if (id == 0)
    {
        yCInfo(HICO) << "Clearing filters previously set";

        if (!filterManager->clearFilters(true))
        {
            yCError(HICO) << "Unable to clear accceptance filters:" << std::strerror(errno);
            return false;
        }

        return true;
    }

    if (!filterManager->hasId(id))
    {
        yCWarning(HICO) << "Filter for ID" << id << "not found, doing nothing";
        return true;
    }

    if (!filterManager->eraseId(id))
    {
        yCError(HICO) << "Could not remove filter:" << std::strerror(errno);
        return false;
    }

    if (!filterManager->isValid())
    {
        yCWarning(HICO) << "Hardware limit was hit, not all requested filters are enabled";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    if (!allowPermissive && wait != blockingMode)
    {
        yCError(HICO, "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
        return false;
    }

    *read = 0;

    std::lock_guard<std::mutex> lockGuard(canBusReady);

    for (unsigned int i = 0; i < size; i++)
    {
        if (blockingMode && rxTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(READ, &bufferReady))
            {
                yCError(HICO, "waitUntilTimeout() failed");
                return false;
            }

            if (!bufferReady)
            {
                break;
            }
        }

        yarp::dev::CanMessage & msg = msgs[i];
        struct can_msg * _msg = reinterpret_cast<struct can_msg *>(msg.getPointer());

        //-- read() returns the number of bytes read, -1 for errors, 0 for EOF.
        int ret = ::read(fileDescriptor, _msg, sizeof(struct can_msg));

        if (ret == -1)
        {
            if (!blockingMode && errno == EAGAIN)
            {
                break;
            }
            else
            {
                yCError(HICO, "read() error: %s", std::strerror(errno));
                return false;
            }
        }
        else if (ret == 0)
        {
            break;
        }
        else
        {
            (*read)++;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    if (!allowPermissive && wait != blockingMode)
    {
        yCError(HICO, "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
        return false;
    }

    *sent = 0;

    std::lock_guard<std::mutex> lockGuard(canBusReady);

    for (unsigned int i = 0; i < size; i++)
    {
        if (blockingMode && txTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(WRITE, &bufferReady))
            {
                yCError(HICO, "waitUntilTimeout() failed");
                return false;
            }

            if (!bufferReady)
            {
                break;
            }
        }

        const struct can_msg * _msg = reinterpret_cast<const struct can_msg *>(msgs[i].getPointer());

        //-- write() returns the number of bytes sent or -1 for errors.
        int ret = ::write(fileDescriptor, _msg, sizeof(struct can_msg));

        if (ret == -1)
        {
            if (!blockingMode && errno == EAGAIN)
            {
                break;
            }
            else
            {
                yCError(HICO, "write() failed: %s", std::strerror(errno));
                return false;
            }
        }
        else if (ret == 0)
        {
            break;
        }
        else
        {
            (*sent)++;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------
