// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <sys/ioctl.h>

#include <cstring>
#include <cerrno>

#include <string>

#include <ColorDebug.hpp>

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canSetBaudRate(unsigned int rate)
{
    CD_DEBUG("(%d)\n", rate);

    std::string rateStr;

    if (!interpretBitrate(rate, rateStr))
    {
        CD_ERROR("Unrecognized bitrate value.\n");
        return false;
    }

    canBusReady.wait();

    if (bitrateState.first)
    {
        if (bitrateState.second == rate)
        {
            CD_WARNING("Bitrate already set.\n");
            canBusReady.post();
            return true;
        }
        else
        {
            CD_ERROR("Bitrate already set to a different value: %d.\n", bitrateState.second);
            canBusReady.post();
            return false;
        }
    }

    if (::ioctl(fileDescriptor, IOC_SET_BITRATE, &rate) == -1)
    {
        CD_ERROR("Could not set bitrate: %s.\n", std::strerror(errno));
        canBusReady.post();
        return false;
    }

    bitrateState.first = true;
    bitrateState.second = rate;

    canBusReady.post();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canGetBaudRate(unsigned int * rate)
{
    CD_DEBUG("\n");

    canBusReady.wait();
    int ret = ::ioctl(fileDescriptor, IOC_GET_BITRATE, rate);
    canBusReady.post();

    if (ret == -1)
    {
        CD_ERROR("Could not set bitrate: %s.\n", std::strerror(errno));
        return false;
    }

    std::string rateStr;

    if (interpretBitrate(*rate, rateStr))
    {
        CD_DEBUG("Got bitrate: %s (%d).\n", rateStr.c_str(), *rate);
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

    canBusReady.wait();

    if (filterManager->hasId(id))
    {
        CD_WARNING("Filter for ID %d is already active.\n", id);
        canBusReady.post();
        return true;
    }

    if (!filterManager->insertId(id))
    {
        CD_ERROR("Could not set filter: %s.\n", std::strerror(errno));
        canBusReady.post();
        return false;
    }

    if (!filterManager->isValid())
    {
        CD_WARNING("Hardware limit was hit, not all requested filters are enabled.\n");
        canBusReady.post();
        return true;
    }

    canBusReady.post();

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

    canBusReady.wait();

    if (!filterManager->hasId(id))
    {
        CD_WARNING("Filter for ID %d not found, doing nothing.\n", id);
        canBusReady.post();
        return true;
    }

    if (!filterManager->eraseId(id))
    {
        CD_ERROR("Could not remove filter: %s.\n", std::strerror(errno));
        canBusReady.post();
        return false;
    }

    if (!filterManager->isValid())
    {
        CD_WARNING("Hardware limit was hit, not all requested filters are enabled.\n");
        canBusReady.post();
        return false;
    }

    canBusReady.post();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    *read = 0;

    canBusReady.wait();

    if (!setFdMode(wait))
    {
        CD_ERROR("setFdMode() failed.\n");
        canBusReady.post();
        return false;
    }

    for (unsigned int i = 0; i < size; i++)
    {
        if (wait && rxTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(READ, &bufferReady))
            {
                CD_ERROR("waitUntilTimeout() failed.\n");
                canBusReady.wait();
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
            CD_ERROR("read() error: %s.\n", std::strerror(errno));
            canBusReady.wait();
            return false;
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

    canBusReady.post();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    *sent = 0;

    canBusReady.wait();

    if (!setFdMode(wait))
    {
        CD_ERROR("setFdMode() failed.\n");
        canBusReady.post();
        return false;
    }

    for (unsigned int i = 0; i < size; i++)
    {
        if (wait && txTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(WRITE, &bufferReady))
            {
                CD_ERROR("waitUntilTimeout() failed.\n");
                canBusReady.wait();
                return false;
            }

            if (!bufferReady)
            {
                break;
            }
        }

        const yarp::dev::CanMessage & msg = const_cast<yarp::dev::CanBuffer &>(msgs)[i];
        const struct can_msg * _msg = reinterpret_cast<const struct can_msg *>(msg.getPointer());

        //-- read() returns the number of bytes sent or -1 for errors.
        int ret = ::write(fileDescriptor, _msg, sizeof(struct can_msg));

        if (ret == -1)
        {
            CD_ERROR("%s.\n", std::strerror(errno));
            canBusReady.post();
            return false;
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

    canBusReady.post();

    return true;
}

// -----------------------------------------------------------------------------
