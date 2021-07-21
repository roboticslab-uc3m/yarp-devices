// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusSocket.hpp"

#include <linux/can/raw.h>
#include <sys/socket.h>

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
    yCError(SCK) << "canGetBaudRate() not available";
    return false;
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

    return false;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    if (!allowPermissive && wait != blockingMode)
    {
        yCError(SCK, "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
        return false;
    }

    return false;
}

// -----------------------------------------------------------------------------
