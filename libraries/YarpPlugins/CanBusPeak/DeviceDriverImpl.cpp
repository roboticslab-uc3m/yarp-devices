// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusPeak.hpp"

#include <cstring> // std::strerror

#include <string>

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

constexpr auto DEFAULT_PORT = "/dev/pcan0";
constexpr auto DEFAULT_BITRATE = 1000000;

constexpr auto DEFAULT_RX_TIMEOUT_MS = 1;
constexpr auto DEFAULT_TX_TIMEOUT_MS = 0; // '0' means no timeout

constexpr auto DEFAULT_BLOCKING_MODE = true;
constexpr auto DEFAULT_ALLOW_PERMISSIVE = false;

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool CanBusPeak::open(yarp::os::Searchable& config)
{
#if YARP_VERSION_MINOR < 6
    yCDebug(PEAK) << "Config:" << config.toString();
#endif

    std::string devicePath = config.check("port", yarp::os::Value(DEFAULT_PORT), "CAN device path").asString();

#if YARP_VERSION_MINOR >= 6
    yarp::dev::DeviceDriver::setId(devicePath);
#endif

    int bitrate = config.check("bitrate", yarp::os::Value(DEFAULT_BITRATE), "CAN bitrate (bps)").asInt32();

    blockingMode = config.check("blockingMode", yarp::os::Value(DEFAULT_BLOCKING_MODE), "blocking mode enabled").asBool();
    allowPermissive = config.check("allowPermissive", yarp::os::Value(DEFAULT_ALLOW_PERMISSIVE), "read/write permissive mode").asBool();

    int flags = OFD_BITRATE | PCANFD_INIT_STD_MSG_ONLY;

    if (blockingMode)
    {
#if YARP_VERSION_MINOR >= 6
        yCIInfo(PEAK, id()) << "Blocking mode enabled";
#else
        yCInfo(PEAK) << "Blocking mode enabled for CAN device" << devicePath;
#endif

        rxTimeoutMs = config.check("rxTimeoutMs", yarp::os::Value(DEFAULT_RX_TIMEOUT_MS), "RX timeout (milliseconds)").asInt32();
        txTimeoutMs = config.check("txTimeoutMs", yarp::os::Value(DEFAULT_TX_TIMEOUT_MS), "TX timeout (milliseconds)").asInt32();

        if (rxTimeoutMs <= 0)
        {
#if YARP_VERSION_MINOR >= 6
            yCIWarning(PEAK, id()) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready";
#else
            yCWarning(PEAK) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready";
#endif
        }

        if (txTimeoutMs <= 0)
        {
#if YARP_VERSION_MINOR >= 6
            yCIWarning(PEAK, id()) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready";
#else
            yCWarning(PEAK) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready";
#endif
        }
    }
    else
    {
#if YARP_VERSION_MINOR >= 6
        yCIInfo(PEAK, id()) << "Requested non-blocking mode";
#else
        yCInfo(PEAK) << "Requested non-blocking mode for CAN device" << devicePath;
#endif
        flags |= OFD_NONBLOCKING;
    }

#if YARP_VERSION_MINOR >= 6
    yCIInfo(PEAK, id()) << "Permissive mode flag for read/write operations set to" << allowPermissive;
#else
    yCInfo(PEAK) << "Permissive mode flag for read/write operations on CAN device" << devicePath << "set to" << allowPermissive;
#endif

    int res = pcanfd_open(devicePath.c_str(), flags, bitrate);

    if (res < 0)
    {
#if YARP_VERSION_MINOR >= 6
        yCIError(PEAK, id()) << "Unable to open:" << std::strerror(-res);
#else
        yCError(PEAK, "Unable to open CAN device %s (%s)", devicePath.c_str(), std::strerror(-res));
#endif
        return false;
    }
    else
    {
#if YARP_VERSION_MINOR >= 6
        yCIInfo(PEAK, id()) << "Successfully opened";
#else
        yCInfo(PEAK) << "Successfully opened CAN device" << devicePath;
#endif
        fileDescriptor = res;
    }

    if (!config.check("preserveFilters", "don't clear acceptance filters on init"))
    {
        res = pcanfd_del_filters(fileDescriptor);

        if (res < 0)
        {
#if YARP_VERSION_MINOR >= 6
            yCIError(PEAK, id()) << "Unable to clear acceptance filters:" << std::strerror(-res);
#else
            yCError(PEAK, "Unable to clear acceptance filters on CAN device %s (%s)", devicePath.c_str(), std::strerror(-res));
#endif
            return false;
        }
        else
        {
#if YARP_VERSION_MINOR >= 6
            yCIInfo(PEAK, id()) << "Acceptance filters cleared";
#else
            yCInfo(PEAK) << "Acceptance filters cleared on CAN device" << devicePath;
#endif
        }
    }
    else
    {
#if YARP_VERSION_MINOR >= 6
        yCIWarning(PEAK, id()) << "Preserving previous acceptance filters (if any)";
#else
        yCWarning(PEAK) << "Preserving previous acceptance filters (if any) on" << devicePath;
#endif
    }

    //-- Load initial node IDs and set acceptance filters.
    if (config.check("filteredIds", "filtered node IDs"))
    {
        const yarp::os::Bottle * ids = config.findGroup("filteredIds").get(1).asList();

        if (ids->size() != 0)
        {
#if YARP_VERSION_MINOR >= 6
            yCIInfo(PEAK, id()) << "Parsing bottle of ids";
#else
            yCInfo(PEAK) << "Parsing bottle of ids on CAN device" << devicePath;
#endif

            for (int i = 0; i < ids->size(); i++)
            {
                activeFilters.insert(ids->get(i).asInt32());
            }

            std::uint64_t acc = computeAcceptanceCodeAndMask();

#if YARP_VERSION_MINOR >= 6
            yCIDebug(PEAK, id(), "New acceptance code+mask: %016lxh", acc);
#else
            yCDebug(PEAK, "New acceptance code+mask: %016lxh", acc);
#endif

            res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

            if (res < 0)
            {
#if YARP_VERSION_MINOR >= 6
                yCIError(PEAK, id()) << "Unable to set acceptance filters:" << std::strerror(-res);
#else
                yCError(PEAK, "Unable to set acceptance filters on CAN device: %s (%s)", devicePath.c_str(), std::strerror(-res));
#endif
                activeFilters.clear();
                return false;
            }

#if YARP_VERSION_MINOR >= 6
            yCIInfo(PEAK, id()) << "Initial IDs added to set of acceptance filters";
#else
            yCInfo(PEAK) << "Initial IDs added to set of acceptance filters in CAN device" << devicePath;
#endif
        }
        else
        {
#if YARP_VERSION_MINOR >= 6
            yCIInfo(PEAK, id()) << "No bottle of ids provided";
#else
            yCInfo(PEAK) << "No bottle of ids given to CAN device" << devicePath;
#endif
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusPeak::close()
{
    if (fileDescriptor > 0)
    {
        if (!activeFilters.empty())
        {
            int res = pcanfd_del_filters(fileDescriptor);

            if (res < 0)
            {
#if YARP_VERSION_MINOR >= 6
                yCIWarning(PEAK, id()) << "Unable to clear acceptance filters:" << std::strerror(-res);
#else
                yCWarning(PEAK) << "Unable to clear acceptance filters:" << std::strerror(-res);
#endif
            }
            else
            {
                activeFilters.clear();
            }
        }

        pcanfd_close(fileDescriptor);
        fileDescriptor = 0;
    }

    return true;
}

// -----------------------------------------------------------------------------
