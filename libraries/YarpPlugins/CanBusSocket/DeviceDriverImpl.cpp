// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusSocket.hpp"

#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

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
    yCDebug(SCK) << "Config:" << config.toString();

    iface = config.check("port", yarp::os::Value(DEFAULT_PORT), "CAN socket interface").asString();
    blockingMode = config.check("blockingMode", yarp::os::Value(DEFAULT_BLOCKING_MODE), "blocking mode enabled").asBool();
    allowPermissive = config.check("allowPermissive", yarp::os::Value(DEFAULT_ALLOW_PERMISSIVE), "read/write permissive mode").asBool();
    bitrate = config.check("bitrate", yarp::os::Value(0), "CAN bitrate (bps)").asInt32();

    if (blockingMode)
    {
        rxTimeoutMs = config.check("rxTimeoutMs", yarp::os::Value(DEFAULT_RX_TIMEOUT_MS), "CAN RX timeout (milliseconds)").asInt32();
        txTimeoutMs = config.check("txTimeoutMs", yarp::os::Value(DEFAULT_TX_TIMEOUT_MS), "CAN TX timeout (milliseconds)").asInt32();

        if (rxTimeoutMs <= 0)
        {
            yCWarning(SCK) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready:" << iface;
        }

        if (txTimeoutMs <= 0)
        {
            yCWarning(SCK) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready:" << iface;
        }
    }

    yCInfo(SCK) << "Permissive mode flag for read/write operations on iface" << iface << "set to" << allowPermissive;

    if ((s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        yCError(SCK) << "Unable to open socket for iface" << iface << "with error:" << std::strerror(errno);
        return false;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, iface.c_str());

    if (::ioctl(s, SIOCGIFINDEX, &ifr) < 0)
    {
        yCError(SCK) << "Unable to determine index for iface" << iface << "with error:" << std::strerror(errno);
        return false;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        yCError(SCK) << "Unable to bind address for iface" << iface << "with error:" << std::strerror(errno);
        return false;
    }

    if (!blockingMode)
    {
        int fcntlFlags;

        if ((fcntlFlags = ::fcntl(s, F_GETFL)) < 0)
        {
            yCError(SCK) << "Unable to retrieve FD flags for iface" << iface << "with error:" << std::strerror(errno);
            return false;
        }

        if (::fcntl(s, F_SETFL, fcntlFlags | O_NONBLOCK) < 0)
        {
            yCError(SCK) << "Unable to set non-blocking mode for iface" << iface << "with error:" << std::strerror(errno);
            return false;
        }

        yCInfo(SCK) << "Non-blocking mode enabled for iface" << iface;
    }
    else
    {
        yCInfo(SCK) << "Blocking mode enabled for iface" << iface;
    }

    can_err_mask_t errorMask = CAN_ERR_MASK;

    if (::setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errorMask, sizeof(errorMask)) < 0)
    {
        yCError(SCK) << "Unable to enable error frames for iface" << iface << "with error:" << std::strerror(errno);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::close()
{
    if (s > 0 && ::close(s) < 0)
    {
        yCError(SCK) << "Unable to close socket for iface" << iface << "with error:" << std::strerror(errno);
        return false;
    }

    s = 0;
    filters.clear();
    errors = yarp::dev::CanErrors(); // reset

    return true;
}

// -----------------------------------------------------------------------------
