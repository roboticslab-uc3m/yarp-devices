// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <sys/ioctl.h>

#include <cstring>
#include <cerrno>

#include <string>

#include <yarp/conf/version.h>
#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusHico::canSetBaudRate(unsigned int rate)
{
    unsigned int _id;

    if (!bitrateToId(rate, &_id))
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id()) << "Unsupported bitrate value:" << rate;
#else
        yCError(HICO) << "Unsupported bitrate value:" << rate;
#endif
        return false;
    }

    std::lock_guard<std::mutex> lockGuard(canBusReady);

    if (bitrateState.first)
    {
        if (bitrateState.second == _id)
        {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
            yCIWarning(HICO, id()) << "Bitrate already set";
#else
            yCWarning(HICO) << "Bitrate already set";
#endif
            return true;
        }
        else
        {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
            yCIError(HICO, id()) << "Bitrate already set to a different value:" << bitrateState.second;
#else
            yCError(HICO) << "Bitrate already set to a different value:" << bitrateState.second;
#endif
            return false;
        }
    }

    if (::ioctl(fileDescriptor, IOC_SET_BITRATE, &_id) == -1)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id()) << "Could not set bitrate:" << std::strerror(errno);
#else
        yCError(HICO) << "Could not set bitrate:" << std::strerror(errno);
#endif
        return false;
    }

    bitrateState.first = true;
    bitrateState.second = _id;

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
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id()) << "Could not get bitrate:" << std::strerror(errno);
#else
        yCError(HICO) << "Could not get bitrate:" << std::strerror(errno);
#endif
        return false;
    }

    if (!idToBitrate(_id, rate))
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id()) << "Unrecognized bitrate id:" << _id;
#else
        yCError(HICO) << "Unrecognized bitrate id:" << _id;
#endif
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::canIdAdd(unsigned int _id)
{
    if (filterConfig == FilterManager::DISABLED)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIWarning(HICO, id()) << "CAN filters are not enabled in this device";
#else
        yCWarning(HICO) << "CAN filters are not enabled in this device";
#endif
        return true;
    }

    if (_id > 0x7F)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id(), "Invalid ID (%d > 0x7F)", _id);
#else
        yCError(HICO, "Invalid ID (%d > 0x7F)", _id);
#endif
        return false;
    }

    std::lock_guard<std::mutex> lockGuard(canBusReady);

    if (filterManager->hasId(_id))
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIWarning(HICO, id()) << "Filter for ID" << _id << "is already active";
#else
        yCWarning(HICO) << "Filter for ID" << _id << "is already active";
#endif
        return true;
    }

    if (!filterManager->insertId(_id))
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id()) << "Could not set filter:" << std::strerror(errno);
#else
        yCError(HICO) << "Could not set filter:" << std::strerror(errno);
#endif
        return false;
    }

    if (!filterManager->isValid())
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIWarning(HICO, id()) << "Hardware limit was hit, not all requested filters are enabled";
#else
        yCWarning(HICO) << "Hardware limit was hit, not all requested filters are enabled";
#endif
        return true;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::canIdDelete(unsigned int _id)
{
    if (filterConfig == FilterManager::DISABLED)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIWarning(HICO, id()) << "CAN filters are not enabled in this device";
#else
        yCWarning(HICO) << "CAN filters are not enabled in this device";
#endif
        return true;
    }

    if (_id > 0x7F)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id(), "Invalid ID (%d > 0x7F)", _id);
#else
        yCError(HICO, "Invalid ID (%d > 0x7F)", _id);
#endif
        return false;
    }

    std::lock_guard<std::mutex> lockGuard(canBusReady);

    if (_id == 0)
    {
        yCInfo(HICO) << "Clearing filters previously set";

        if (!filterManager->clearFilters(true))
        {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
            yCIError(HICO, id()) << "Unable to clear accceptance filters:" << std::strerror(errno);
#else
            yCError(HICO) << "Unable to clear accceptance filters:" << std::strerror(errno);
#endif
            return false;
        }

        return true;
    }

    if (!filterManager->hasId(_id))
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIWarning(HICO, id()) << "Filter for ID" << _id << "not found, doing nothing";
#else
        yCWarning(HICO) << "Filter for ID" << _id << "not found, doing nothing";
#endif
        return true;
    }

    if (!filterManager->eraseId(_id))
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id()) << "Could not remove filter:" << std::strerror(errno);
#else
        yCError(HICO) << "Could not remove filter:" << std::strerror(errno);
#endif
        return false;
    }

    if (!filterManager->isValid())
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIWarning(HICO, id()) << "Hardware limit was hit, not all requested filters are enabled";
#else
        yCWarning(HICO) << "Hardware limit was hit, not all requested filters are enabled";
#endif
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    if (!allowPermissive && wait != blockingMode)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id(), "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
#else
        yCError(HICO, "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
#endif
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
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
                yCIError(HICO, id(), "waitUntilTimeout() failed");
#else
                yCError(HICO, "waitUntilTimeout() failed");
#endif
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
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
                yCIError(HICO, id(), "read() error: %s", std::strerror(errno));
#else
                yCError(HICO, "read() error: %s", std::strerror(errno));
#endif
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
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id(), "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
#else
        yCError(HICO, "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
#endif
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
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
                yCIError(HICO, id(), "waitUntilTimeout() failed");
#else
                yCError(HICO, "waitUntilTimeout() failed");
#endif
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
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
                yCIError(HICO, id(), "write() failed: %s", std::strerror(errno));
#else
                yCError(HICO, "write() failed: %s", std::strerror(errno));
#endif
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
