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
#include <yarp/os/Property.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_PORT = "can0";
constexpr auto DEFAULT_BLOCKING_MODE = true;
constexpr auto DEFAULT_ALLOW_PERMISSIVE = false;
constexpr auto DEFAULT_RX_TIMEOUT_MS = 1;
constexpr auto DEFAULT_TX_TIMEOUT_MS = 0; // '0' means no timeout

// ------------------- DeviceDriver Related ------------------------------------

bool CanBusSocket::open(yarp::os::Searchable & config)
{
    yarp::os::Property options;
    options.fromString(config.findGroup("common").toString());
    options.fromString(config.toString(), false); // override common options

    iface = options.check("port", yarp::os::Value(DEFAULT_PORT), "CAN socket interface").asString();
    blockingMode = options.check("blockingMode", yarp::os::Value(DEFAULT_BLOCKING_MODE), "blocking mode enabled").asBool();
    allowPermissive = options.check("allowPermissive", yarp::os::Value(DEFAULT_ALLOW_PERMISSIVE), "read/write permissive mode").asBool();
    filterFunctionCodes = options.check("filterFunctionCodes", yarp::os::Value(true), "filter mask ignores CANopen function codes").asBool();
    bitrate = options.check("bitrate", yarp::os::Value(0), "CAN bitrate (bps)").asInt32();

    if (blockingMode)
    {
        rxTimeoutMs = options.check("rxTimeoutMs", yarp::os::Value(DEFAULT_RX_TIMEOUT_MS), "CAN RX timeout (milliseconds)").asInt32();
        txTimeoutMs = options.check("txTimeoutMs", yarp::os::Value(DEFAULT_TX_TIMEOUT_MS), "CAN TX timeout (milliseconds)").asInt32();

        if (rxTimeoutMs <= 0)
        {
            yCIWarning(SCK, id()) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready";
        }

        if (txTimeoutMs <= 0)
        {
            yCIWarning(SCK, id()) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready";
        }
    }

    yCIInfo(SCK, id()) << "Permissive mode flag for read/write operations set to" << allowPermissive;

    if ((s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        yCIError(SCK, id()) << "Unable to open socket with error:" << std::strerror(errno);
        return false;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, iface.c_str());

    if (::ioctl(s, SIOCGIFINDEX, &ifr) < 0)
    {
        yCIError(SCK, id()) << "Unable to determine index with error:" << std::strerror(errno);
        return false;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        yCIError(SCK, id()) << "Unable to bind address with error:" << std::strerror(errno);
        return false;
    }

    if (!blockingMode)
    {
        int fcntlFlags;

        if ((fcntlFlags = ::fcntl(s, F_GETFL)) < 0)
        {
            yCIError(SCK, id()) << "Unable to retrieve FD flags with error:" << std::strerror(errno);
            return false;
        }

        if (::fcntl(s, F_SETFL, fcntlFlags | O_NONBLOCK) < 0)
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

    if (::setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errorMask, sizeof(errorMask)) < 0)
    {
        yCIError(SCK, id()) << "Unable to enable error frames with error:" << std::strerror(errno);
        return false;
    }

    if (options.check("filteredIds", "filtered node IDs"))
    {
        const auto * ids = options.findGroup("filteredIds").get(1).asList();

        if (ids->size() != 0)
        {
            for (int i = 0; i < ids->size(); i++)
            {
                struct can_filter filter;
                filter.can_id = ids->get(i).asInt32();
                filter.can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);

                if (filterFunctionCodes)
                {
                    filter.can_mask &= ~0x780; // function codes, e.g. 0x580, are ignored in CANopen mode
                }

                filters.push_back(std::move(filter));
            }

            if (::setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(struct can_filter) * filters.size()) < 0)
            {
                yCIError(SCK, id()) << "Unable to configure set of initial acceptance filters";
                filters.clear();
                return false;
            }

            yCIInfo(SCK, id()) << "Initial IDs added to set of acceptance filters";
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::close()
{
    if (s > 0 && ::close(s) < 0)
    {
        yCIError(SCK, id()) << "Unable to close socket with error:" << std::strerror(errno);
        return false;
    }

    s = 0;
    filters.clear();
    return true;
}

// -----------------------------------------------------------------------------
