// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusSocket.hpp"

#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include <utility>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool CanBusSocket::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        yCIError(SCK, id()) << "Could not parse parameters";
        return false;
    }

    yarp::dev::DeviceDriver::setId(m_port);

    if (m_blockingMode)
    {
        if (m_rxTimeoutMs <= 0)
        {
            yCIWarning(SCK, id()) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready";
        }

        if (m_txTimeoutMs <= 0)
        {
            yCIWarning(SCK, id()) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready";
        }
    }

    if ((socketDescriptor = ::socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        yCIError(SCK, id()) << "Unable to open socket with error:" << std::strerror(errno);
        return false;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, m_port.c_str());

    if (::ioctl(socketDescriptor, SIOCGIFINDEX, &ifr) < 0)
    {
        yCIError(SCK, id()) << "Unable to determine index with error:" << std::strerror(errno);
        return false;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(socketDescriptor, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        yCIError(SCK, id()) << "Unable to bind address with error:" << std::strerror(errno);
        return false;
    }

    if (!m_blockingMode)
    {
        int fcntlFlags;

        if ((fcntlFlags = ::fcntl(socketDescriptor, F_GETFL)) < 0)
        {
            yCIError(SCK, id()) << "Unable to retrieve FD flags with error:" << std::strerror(errno);
            return false;
        }

        if (::fcntl(socketDescriptor, F_SETFL, fcntlFlags | O_NONBLOCK) < 0)
        {
            yCIError(SCK, id()) << "Unable to set non-blocking mode with error:" << std::strerror(errno);
            return false;
        }

        yCIInfo(SCK, id()) << "Non-blocking mode enabled";
    }
    else
    {
        yCIInfo(SCK, id()) << "Blocking mode enabled";
    }

    can_err_mask_t errorMask = CAN_ERR_MASK;

    if (::setsockopt(socketDescriptor, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errorMask, sizeof(errorMask)) < 0)
    {
        yCIError(SCK, id()) << "Unable to enable error frames with error:" << std::strerror(errno);
        return false;
    }

    if (!m_filteredIds.empty())
    {
        for (const auto id : m_filteredIds)
        {
            struct can_filter filter;
            filter.can_id = id;
            filter.can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);

            if (m_filterFunctionCodes)
            {
                filter.can_mask &= ~0x780; // function codes, e.g. 0x580, are ignored in CANopen mode
            }

            filters.push_back(std::move(filter));
        }

        if (::setsockopt(socketDescriptor, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(struct can_filter) * filters.size()) < 0)
        {
            yCIError(SCK, id()) << "Unable to configure set of initial acceptance filters";
            filters.clear();
            return false;
        }

        yCIInfo(SCK, id()) << "Initial IDs added to set of acceptance filters";
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::close()
{
    if (socketDescriptor > 0 && ::close(socketDescriptor) < 0)
    {
        yCIError(SCK, id()) << "Unable to close socket with error:" << std::strerror(errno);
        return false;
    }

    socketDescriptor = 0;
    filters.clear();
    return true;
}

// -----------------------------------------------------------------------------
