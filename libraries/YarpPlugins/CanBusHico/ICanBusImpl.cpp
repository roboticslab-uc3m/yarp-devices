// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <sys/ioctl.h>

#include <cstring>
#include <cerrno>

#include <string>

#include <yarp/os/LockGuard.h>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canSetBaudRate(unsigned int rate)
{
    CD_DEBUG("(%d)\n", rate);

    unsigned int id;

    if (!bitrateToId(rate, &id))
    {
        CD_ERROR("Unsupported bitrate value (%d).\n", rate);
        return false;
    }

    yarp::os::LockGuard lockGuard(canBusReady);

    if (bitrateState.first)
    {
        if (bitrateState.second == id)
        {
            CD_WARNING("Bitrate already set.\n");
            return true;
        }
        else
        {
            CD_ERROR("Bitrate already set to a different value: %d.\n", bitrateState.second);
            return false;
        }
    }

    if (::ioctl(fileDescriptor, IOC_SET_BITRATE, &id) == -1)
    {
        CD_ERROR("Could not set bitrate: %s.\n", std::strerror(errno));
        return false;
    }

    bitrateState.first = true;
    bitrateState.second = id;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canGetBaudRate(unsigned int * rate)
{
    CD_DEBUG("\n");

    unsigned int id;

    canBusReady.lock();
    int ret = ::ioctl(fileDescriptor, IOC_GET_BITRATE, &id);
    canBusReady.unlock();

    if (ret == -1)
    {
        CD_ERROR("Could not set bitrate: %s.\n", std::strerror(errno));
        return false;
    }

    if (!idToBitrate(id, rate))
    {
        CD_ERROR("Unrecognized bitrate id (%d).\n", id);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canIdAdd(unsigned int id)
{
    CD_DEBUG("(%d)\n", id);

    if (filterConfig == FilterManager::DISABLED)
    {
        CD_WARNING("CAN filters are not enabled in this device.\n");
        return true;
    }

    if (id > 0x7F)
    {
        CD_ERROR("Invalid ID (%d > 0x7F).\n", id);
        return false;
    }

    yarp::os::LockGuard lockGuard(canBusReady);

    if (filterManager->hasId(id))
    {
        CD_WARNING("Filter for ID %d is already active.\n", id);
        return true;
    }

    if (!filterManager->insertId(id))
    {
        CD_ERROR("Could not set filter: %s.\n", std::strerror(errno));
        return false;
    }

    if (!filterManager->isValid())
    {
        CD_WARNING("Hardware limit was hit, not all requested filters are enabled.\n");
        return true;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canIdDelete(unsigned int id)
{
    CD_DEBUG("(%d)\n", id);

    if (filterConfig == FilterManager::DISABLED)
    {
        CD_WARNING("CAN filters are not enabled in this device.\n");
        return true;
    }

    if (id > 0x7F)
    {
        CD_ERROR("Invalid ID (%d > 0x7F).\n", id);
        return false;
    }

    yarp::os::LockGuard lockGuard(canBusReady);

    if (id == 0)
    {
        CD_INFO("Clearing filters previously set.\n");

        if (!filterManager->clearFilters(true))
        {
            CD_ERROR("Unable to clear accceptance filters: %s.\n", std::strerror(errno));
            return false;
        }

        return true;
    }

    if (!filterManager->hasId(id))
    {
        CD_WARNING("Filter for ID %d not found, doing nothing.\n", id);
        return true;
    }

    if (!filterManager->eraseId(id))
    {
        CD_ERROR("Could not remove filter: %s.\n", std::strerror(errno));
        return false;
    }

    if (!filterManager->isValid())
    {
        CD_WARNING("Hardware limit was hit, not all requested filters are enabled.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    if (!allowPermissive && wait != blockingMode)
    {
        CD_ERROR("Blocking mode configuration mismatch: requested=%d, enabled=%d.\n", wait, blockingMode);
        return false;
    }

    *read = 0;

    yarp::os::LockGuard lockGuard(canBusReady);

    for (unsigned int i = 0; i < size; i++)
    {
        if (blockingMode && rxTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(READ, &bufferReady))
            {
                CD_ERROR("waitUntilTimeout() failed.\n");
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
                CD_ERROR("read() error: %s.\n", std::strerror(errno));
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

bool roboticslab::CanBusHico::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    if (!allowPermissive && wait != blockingMode)
    {
        CD_ERROR("Blocking mode configuration mismatch: requested=%d, enabled=%d.\n", wait, blockingMode);
        return false;
    }

    *sent = 0;

    yarp::os::LockGuard lockGuard(canBusReady);

    for (unsigned int i = 0; i < size; i++)
    {
        if (blockingMode && txTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(WRITE, &bufferReady))
            {
                CD_ERROR("waitUntilTimeout() failed.\n");
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
                CD_ERROR("%s.\n", std::strerror(errno));
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
