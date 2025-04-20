// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusSocket.hpp"

#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include <algorithm>
#include <utility>

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
    if (m_bitrate == 0)
    {
        yCIError(SCK, id()) << "Bitrate not available";
        return false;
    }

    *rate = m_bitrate;
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canIdAdd(unsigned int _id)
{
    struct can_filter filter;
    filter.can_id = _id;
    filter.can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);

    if (m_filterFunctionCodes)
    {
        filter.can_mask &= ~0x780; // function codes, e.g. 0x580, are ignored in CANopen mode
    }

    if (std::find_if(filters.begin(), filters.end(), [_id](const auto & f) { return f.can_id == _id; }) != filters.end())
    {
        yCIWarning(SCK, id()) << "Filter for id" << _id << "already set";
        return true;
    }

    filters.push_back(std::move(filter));

    if (::setsockopt(socketDescriptor, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(struct can_filter) * filters.size()) < 0)
    {
        yCIError(SCK, id()) << "Unable to add filter for id" << _id;
        filters.pop_back();
        return false;
    }

    yCIInfo(SCK, id()) << "Added filter for id" << _id;

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canIdDelete(unsigned int _id)
{
    auto it = std::find_if(filters.begin(), filters.end(), [_id](const auto & f) { return f.can_id == _id; });

    if (_id == 0 || it != filters.end() && filters.size() == 1)
    {
        struct can_filter filter;
        filter.can_id = 0 | CAN_INV_FILTER;
        filter.can_mask = CAN_SFF_MASK;

        if (::setsockopt(socketDescriptor, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0)
        {
            yCIError(SCK, id()) << "Unable to reset filters";
            return false;
        }

        filters.clear();
    }
    else
    {
        if (it == filters.end())
        {
            yCIWarning(SCK, id()) << "Filter for id" << _id << "missing or already deleted";
            return true;
        }

        if (::setsockopt(socketDescriptor, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0) < 0)
        {
            yCIError(SCK, id()) << "Unable to clear filters";
            return false;
        }

        filters.erase(it);

        if (::setsockopt(socketDescriptor, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(struct can_filter) * filters.size()) < 0)
        {
            yCIError(SCK, id()) << "Unable to add filters back while deleting id" << _id;
            filters.clear();
            return false;
        }
    }

    yCIInfo(SCK, id()) << "Deleted filter for id" << _id;

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    if (!m_allowPermissive && wait != m_blockingMode)
    {
        yCIError(SCK, id(), "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, m_blockingMode);
        return false;
    }

    *read = 0;

    if (m_blockingMode && m_rxTimeoutMs > 0)
    {
        bool bufferReady;

        if (!waitUntilTimeout(READ, &bufferReady))
        {
            yCIError(SCK, id(), "waitUntilTimeout() failed");
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
        int ret = ::read(socketDescriptor, _msg, sizeof(struct can_frame));

        if (ret == -1)
        {
            if (!m_blockingMode && errno == EAGAIN)
            {
                break;
            }
            else
            {
                yCIError(SCK, id(), "read() error: %s", std::strerror(errno));
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
    if (!m_allowPermissive && wait != m_blockingMode)
    {
        yCIError(SCK, id(), "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, m_blockingMode);
        return false;
    }

    *sent = 0;

    if (m_blockingMode && m_txTimeoutMs > 0)
    {
        bool bufferReady;

        if (!waitUntilTimeout(WRITE, &bufferReady))
        {
            yCIError(SCK, id(), "waitUntilTimeout() failed");
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
        int ret = ::write(socketDescriptor, _msg, sizeof(struct can_frame));

        if (ret == -1)
        {
            if (!m_blockingMode && errno == EAGAIN)
            {
                break;
            }
            else
            {
                yCIError(SCK, id(), "write() failed: %s", std::strerror(errno));
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
