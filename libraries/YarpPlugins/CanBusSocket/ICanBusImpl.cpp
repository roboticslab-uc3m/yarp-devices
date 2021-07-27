// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusSocket.hpp"

#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include <algorithm>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusSocket::canSetBaudRate(unsigned int rate)
{
    yCError(SCK) << "canSetBaudRate() not available";
    return false;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canGetBaudRate(unsigned int * rate)
{
    if (bitrate == 0)
    {
        yCError(SCK) << "Bitrate not available for iface" << iface;
        return false;
    }

    *rate = bitrate;
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canIdAdd(unsigned int id)
{
    struct can_filter filter;
    filter.can_id = id;
    filter.can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);

    if (std::find_if(filters.begin(), filters.end(), [id](const auto & f) { return f.can_id == id; }) != filters.end())
    {
        yCWarning(SCK) << "Filter for id" << id << "already set in iface" << iface;
        return true;
    }

    filters.push_back(filter);

    if (::setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(struct can_filter) * filters.size()) < 0)
    {
        yCError(SCK) << "Unable to add filter for id" << id << "at iface" << iface;
        filters.pop_back();
        return false;
    }

    yCInfo(SCK) << "Added filter for id" << id << "at iface" << iface;

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canIdDelete(unsigned int id)
{
    auto it = std::find_if(filters.begin(), filters.end(), [id](const auto & f) { return f.can_id == id; });

    if (id == 0 || it != filters.end() && filters.size() == 1)
    {
        struct can_filter filter;
        filter.can_id = 0 | CAN_INV_FILTER;
        filter.can_mask = CAN_SFF_MASK;

        if (::setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0)
        {
            yCError(SCK) << "Unable to reset filters at iface" << iface;
            return false;
        }

        filters.clear();
    }
    else
    {
        if (it == filters.end())
        {
            yCWarning(SCK) << "Filter for id" << id << "missing or already deleted in iface" << iface;
            return true;
        }

        if (::setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0) < 0)
        {
            yCError(SCK) << "Unable to clear filters at iface" << iface;
            return false;
        }

        filters.erase(it);

        if (::setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(struct can_filter) * filters.size()) < 0)
        {
            yCError(SCK) << "Unable to add filters back while deleting id" << id << "at iface" << iface;
            filters.clear();
            return false;
        }
    }

    yCInfo(SCK) << "Deleted filter for id" << id << "at iface" << iface;

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    if (!allowPermissive && wait != blockingMode)
    {
        yCError(SCK, "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
        return false;
    }

    *read = 0;

    if (blockingMode && rxTimeoutMs > 0)
    {
        bool bufferReady;

        if (!waitUntilTimeout(READ, &bufferReady))
        {
            yCError(SCK, "waitUntilTimeout() failed");
            return false;
        }

        if (!bufferReady)
        {
            return true;
        }
    }

    for (unsigned int i = 0; i < size; i++)
    {
        auto * _msg = reinterpret_cast<struct can_frame *>(msgs[i].getPointer());

        //-- read() returns the number of bytes read, -1 for errors, 0 for EOF.
        int ret = ::read(s, _msg, sizeof(struct can_frame));

        if (ret == -1)
        {
            if (!blockingMode && errno == EAGAIN)
            {
                break;
            }
            else
            {
                yCError(SCK, "read() error: %s", std::strerror(errno));
                return false;
            }
        }
        else if (ret == 0)
        {
            break;
        }
        else if (_msg->can_id & CAN_ERR_FLAG)
        {
            interpretErrorFrame(_msg);
            // same CAN message object will be pulled again from the `msgs` buffer and overwritten
            size--;
            i--;
        }
        else
        {
            (*read)++;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    if (!allowPermissive && wait != blockingMode)
    {
        yCError(SCK, "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
        return false;
    }

    *sent = 0;

    if (blockingMode && txTimeoutMs > 0)
    {
        bool bufferReady;

        if (!waitUntilTimeout(WRITE, &bufferReady))
        {
            yCError(SCK, "waitUntilTimeout() failed");
            return false;
        }

        if (!bufferReady)
        {
            return true;
        }
    }

    for (unsigned int i = 0; i < size; i++)
    {
        const auto * _msg = reinterpret_cast<const struct can_frame *>(msgs[i].getPointer());

        //-- write() returns the number of bytes sent or -1 for errors.
        int ret = ::write(s, _msg, sizeof(struct can_frame));

        if (ret == -1)
        {
            if (!blockingMode && errno == EAGAIN)
            {
                break;
            }
            else
            {
                yCError(SCK, "write() failed: %s", std::strerror(errno));
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
