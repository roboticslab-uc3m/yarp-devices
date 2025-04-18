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
    unsigned int _id;

    if (!bitrateToId(rate, &_id))
    {
        yCIError(HICO, id()) << "Unsupported bitrate value:" << rate;
        return false;
    }

    std::lock_guard lockGuard(canBusReady);

    if (bitrateState.first)
    {
        if (bitrateState.second == _id)
        {
            yCIWarning(HICO, id()) << "Bitrate already set";
            return true;
        }
        else
        {
            yCIError(HICO, id()) << "Bitrate already set to a different value:" << bitrateState.second;
            return false;
        }
    }

    if (::ioctl(fileDescriptor, IOC_SET_BITRATE, &_id) == -1)
    {
        yCIError(HICO, id()) << "Could not set bitrate:" << std::strerror(errno);
        return false;
    }

    bitrateState = {true, _id};

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::canGetBaudRate(unsigned int * rate)
{
    unsigned int _id;

    canBusReady.lock();
    int ret = ::ioctl(fileDescriptor, IOC_GET_BITRATE, &_id);
    canBusReady.unlock();

    if (ret == -1)
    {
        yCIError(HICO, id()) << "Could not get bitrate:" << std::strerror(errno);
        return false;
    }

    if (!idToBitrate(_id, rate))
    {
        yCIError(HICO, id()) << "Unrecognized bitrate id:" << _id;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::canIdAdd(unsigned int _id)
{
    if (filterConfig == FilterManager::DISABLED)
    {
        yCIWarning(HICO, id()) << "CAN filters are not enabled in this device";
        return true;
    }

    if (_id > 0x7F)
    {
        yCIError(HICO, id(), "Invalid ID (%d > 0x7F)", _id);
        return false;
    }

    std::lock_guard lockGuard(canBusReady);

    if (filterManager->hasId(_id))
    {
        yCIWarning(HICO, id()) << "Filter for ID" << _id << "is already active";
        return true;
    }

    if (!filterManager->insertId(_id))
    {
        yCIError(HICO, id()) << "Could not set filter:" << std::strerror(errno);
        return false;
    }

    if (!filterManager->isValid())
    {
        yCIWarning(HICO, id()) << "Hardware limit was hit, not all requested filters are enabled";
        return true;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::canIdDelete(unsigned int _id)
{
    if (filterConfig == FilterManager::DISABLED)
    {
        yCIWarning(HICO, id()) << "CAN filters are not enabled in this device";
        return true;
    }

    if (_id > 0x7F)
    {
        yCIError(HICO, id(), "Invalid ID (%d > 0x7F)", _id);
        return false;
    }

    std::lock_guard lockGuard(canBusReady);

    if (_id == 0)
    {
        yCInfo(HICO) << "Clearing filters previously set";

        if (!filterManager->clearFilters(true))
        {
            yCIError(HICO, id()) << "Unable to clear accceptance filters:" << std::strerror(errno);
            return false;
        }

        return true;
    }

    if (!filterManager->hasId(_id))
    {
        yCIWarning(HICO, id()) << "Filter for ID" << _id << "not found, doing nothing";
        return true;
    }

    if (!filterManager->eraseId(_id))
    {
        yCIError(HICO, id()) << "Could not remove filter:" << std::strerror(errno);
        return false;
    }

    if (!filterManager->isValid())
    {
        yCIWarning(HICO, id()) << "Hardware limit was hit, not all requested filters are enabled";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    if (!m_allowPermissive && wait != m_blockingMode)
    {
        yCIError(HICO, id(), "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, m_blockingMode);
        return false;
    }

    *read = 0;

    std::lock_guard lockGuard(canBusReady);

    for (unsigned int i = 0; i < size; i++)
    {
        if (m_blockingMode && m_rxTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(READ, &bufferReady))
            {
                yCIError(HICO, id(), "waitUntilTimeout() failed");
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
            if (!m_blockingMode && errno == EAGAIN)
            {
                break;
            }
            else
            {
                yCIError(HICO, id(), "read() error: %s", std::strerror(errno));
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
    if (!m_allowPermissive && wait != m_blockingMode)
    {
        yCIError(HICO, id(), "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, m_blockingMode);
        return false;
    }

    *sent = 0;

    std::lock_guard lockGuard(canBusReady);

    for (unsigned int i = 0; i < size; i++)
    {
        if (m_blockingMode && m_txTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(WRITE, &bufferReady))
            {
                yCIError(HICO, id(), "waitUntilTimeout() failed");
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
            if (!m_blockingMode && errno == EAGAIN)
            {
                break;
            }
            else
            {
                yCIError(HICO, id(), "write() failed: %s", std::strerror(errno));
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
