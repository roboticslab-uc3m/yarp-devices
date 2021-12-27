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

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_PORT = "can0";
constexpr auto DEFAULT_BLOCKING_MODE = true;
constexpr auto DEFAULT_ALLOW_PERMISSIVE = false;
constexpr auto DEFAULT_RX_TIMEOUT_MS = 1;
constexpr auto DEFAULT_TX_TIMEOUT_MS = 0; // '0' means no timeout

// ------------------- DeviceDriver Related ------------------------------------

bool CanBusSocket::open(yarp::os::Searchable& config)
{
#if !defined(YARP_VERSION_COMPARE) // < 3.6.0
    yCDebug(SCK) << "Config:" << config.toString();
#endif

    iface = config.check("port", yarp::os::Value(DEFAULT_PORT), "CAN socket interface").asString();
    blockingMode = config.check("blockingMode", yarp::os::Value(DEFAULT_BLOCKING_MODE), "blocking mode enabled").asBool();
    allowPermissive = config.check("allowPermissive", yarp::os::Value(DEFAULT_ALLOW_PERMISSIVE), "read/write permissive mode").asBool();
    filterFunctionCodes = config.check("filterFunctionCodes", yarp::os::Value(true), "filter mask ignores CANopen function codes").asBool();
    bitrate = config.check("bitrate", yarp::os::Value(0), "CAN bitrate (bps)").asInt32();

#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yarp::dev::DeviceDriver::setId(iface);
#endif

    if (blockingMode)
    {
        rxTimeoutMs = config.check("rxTimeoutMs", yarp::os::Value(DEFAULT_RX_TIMEOUT_MS), "CAN RX timeout (milliseconds)").asInt32();
        txTimeoutMs = config.check("txTimeoutMs", yarp::os::Value(DEFAULT_TX_TIMEOUT_MS), "CAN TX timeout (milliseconds)").asInt32();

        if (rxTimeoutMs <= 0)
        {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
            yCIWarning(SCK, id()) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready";
#else
            yCWarning(SCK) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready:" << iface;
#endif
        }

        if (txTimeoutMs <= 0)
        {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
            yCIWarning(SCK, id()) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready";
#else
            yCWarning(SCK) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready:" << iface;
#endif
        }
    }

#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCIInfo(SCK, id()) << "Permissive mode flag for read/write operations set to" << allowPermissive;
#else
    yCInfo(SCK) << "Permissive mode flag for read/write operations on iface" << iface << "set to" << allowPermissive;
#endif

    if ((s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(SCK, id()) << "Unable to open socket with error:" << std::strerror(errno);
#else
        yCError(SCK) << "Unable to open socket for iface" << iface << "with error:" << std::strerror(errno);
#endif
        return false;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, iface.c_str());

    if (::ioctl(s, SIOCGIFINDEX, &ifr) < 0)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(SCK, id()) << "Unable to determine index with error:" << std::strerror(errno);
#else
        yCError(SCK) << "Unable to determine index for iface" << iface << "with error:" << std::strerror(errno);
#endif
        return false;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(SCK, id()) << "Unable to bind address with error:" << std::strerror(errno);
#else
        yCError(SCK) << "Unable to bind address for iface" << iface << "with error:" << std::strerror(errno);
#endif
        return false;
    }

    if (!blockingMode)
    {
        int fcntlFlags;

        if ((fcntlFlags = ::fcntl(s, F_GETFL)) < 0)
        {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
            yCIError(SCK, id()) << "Unable to retrieve FD flags with error:" << std::strerror(errno);
#else
            yCError(SCK) << "Unable to retrieve FD flags for iface" << iface << "with error:" << std::strerror(errno);
#endif
            return false;
        }

        if (::fcntl(s, F_SETFL, fcntlFlags | O_NONBLOCK) < 0)
        {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
            yCIError(SCK, id()) << "Unable to set non-blocking mode with error:" << std::strerror(errno);
#else
            yCError(SCK) << "Unable to set non-blocking mode for iface" << iface << "with error:" << std::strerror(errno);
#endif
            return false;
        }

#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIInfo(SCK, id()) << "Non-blocking mode enabled";
#else
        yCInfo(SCK) << "Non-blocking mode enabled for iface" << iface;
#endif
    }
    else
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIInfo(SCK, id()) << "Blocking mode enabled";
#else
        yCInfo(SCK) << "Blocking mode enabled for iface" << iface;
#endif
    }

    can_err_mask_t errorMask = CAN_ERR_MASK;

    if (::setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errorMask, sizeof(errorMask)) < 0)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(SCK, id()) << "Unable to enable error frames with error:" << std::strerror(errno);
#else
        yCError(SCK) << "Unable to enable error frames for iface" << iface << "with error:" << std::strerror(errno);
#endif
        return false;
    }

    if (config.check("filteredIds", "filtered node IDs"))
    {
        const auto * ids = config.findGroup("filteredIds").get(1).asList();

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
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
                yCIError(SCK, id()) << "Unable to configure set of initial acceptance filters";
#else
                yCError(SCK) << "Unable to configure set of initial acceptance filters at iface" << iface;
#endif
                filters.clear();
                return false;
            }

#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
            yCIInfo(SCK, id()) << "Initial IDs added to set of acceptance filters";
#else
            yCInfo(SCK) << "Initial IDs added to set of acceptance filters for iface" << iface;
#endif
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::close()
{
    if (s > 0 && ::close(s) < 0)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(SCK, id()) << "Unable to close socket with error:" << std::strerror(errno);
#else
        yCError(SCK) << "Unable to close socket for iface" << iface << "with error:" << std::strerror(errno);
#endif
        return false;
    }

    s = 0;
    filters.clear();
    return true;
}

// -----------------------------------------------------------------------------
