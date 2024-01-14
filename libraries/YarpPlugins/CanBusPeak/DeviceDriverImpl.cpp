// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusPeak.hpp"

#include <cstring> // std::strerror

#include <string>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

constexpr auto DEFAULT_PORT = "/dev/pcan0";
constexpr auto DEFAULT_BITRATE = 1000000;

constexpr auto DEFAULT_RX_TIMEOUT_MS = 1;
constexpr auto DEFAULT_TX_TIMEOUT_MS = 0; // '0' means no timeout

constexpr auto DEFAULT_BLOCKING_MODE = true;
constexpr auto DEFAULT_ALLOW_PERMISSIVE = false;
constexpr auto DEFAULT_PRESERVE_FILTERS = false;

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool CanBusPeak::open(yarp::os::Searchable& config)
{
    std::string devicePath = config.check("port", yarp::os::Value(DEFAULT_PORT), "CAN device path").asString();

    yarp::dev::DeviceDriver::setId(devicePath);

    int bitrate = config.check("bitrate", yarp::os::Value(DEFAULT_BITRATE), "CAN bitrate (bps)").asInt32();

    blockingMode = config.check("blockingMode", yarp::os::Value(DEFAULT_BLOCKING_MODE), "blocking mode enabled").asBool();
    allowPermissive = config.check("allowPermissive", yarp::os::Value(DEFAULT_ALLOW_PERMISSIVE), "read/write permissive mode").asBool();

    int flags = OFD_BITRATE | PCANFD_INIT_STD_MSG_ONLY;

    if (blockingMode)
    {
        yCIInfo(PEAK, id()) << "Blocking mode enabled";

        rxTimeoutMs = config.check("rxTimeoutMs", yarp::os::Value(DEFAULT_RX_TIMEOUT_MS), "RX timeout (milliseconds)").asInt32();
        txTimeoutMs = config.check("txTimeoutMs", yarp::os::Value(DEFAULT_TX_TIMEOUT_MS), "TX timeout (milliseconds)").asInt32();

        if (rxTimeoutMs <= 0)
        {
            yCIWarning(PEAK, id()) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready";
        }

        if (txTimeoutMs <= 0)
        {
            yCIWarning(PEAK, id()) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready";
        }
    }
    else
    {
        yCIInfo(PEAK, id()) << "Requested non-blocking mode";
        flags |= OFD_NONBLOCKING;
    }

    yCIInfo(PEAK, id()) << "Permissive mode flag for read/write operations set to" << allowPermissive;

    int res = pcanfd_open(devicePath.c_str(), flags, bitrate);

    if (res < 0)
    {
        yCIError(PEAK, id()) << "Unable to open:" << std::strerror(-res);
        return false;
    }
    else
    {
        yCIInfo(PEAK, id()) << "Successfully opened";
        fileDescriptor = res;
    }

    auto preserveFilters = config.check("preserveFilters", yarp::os::Value(DEFAULT_PRESERVE_FILTERS), "don't clear acceptance filters on init").asBool();

    if (!preserveFilters)
    {
        res = pcanfd_del_filters(fileDescriptor);

        if (res < 0)
        {
            yCIError(PEAK, id()) << "Unable to clear acceptance filters:" << std::strerror(-res);
            return false;
        }
        else
        {
            yCIInfo(PEAK, id()) << "Acceptance filters cleared";
        }
    }
    else
    {
        yCIWarning(PEAK, id()) << "Preserving previous acceptance filters (if any)";
    }

    //-- Load initial node IDs and set acceptance filters.
    if (config.check("filteredIds", "filtered node IDs"))
    {
        const yarp::os::Bottle * ids = config.findGroup("filteredIds").get(1).asList();

        if (ids->size() != 0)
        {
            yCIInfo(PEAK, id()) << "Parsing bottle of ids";

            for (int i = 0; i < ids->size(); i++)
            {
                activeFilters.insert(ids->get(i).asInt32());
            }

            std::uint64_t acc = computeAcceptanceCodeAndMask();

            yCIDebug(PEAK, id(), "New acceptance code+mask: %016lxh", acc);

            res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

            if (res < 0)
            {
                yCIError(PEAK, id()) << "Unable to set acceptance filters:" << std::strerror(-res);
                activeFilters.clear();
                return false;
            }

            yCIInfo(PEAK, id()) << "Initial IDs added to set of acceptance filters";
        }
        else
        {
            yCIInfo(PEAK, id()) << "No bottle of ids provided";
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
                yCIWarning(PEAK, id()) << "Unable to clear acceptance filters:" << std::strerror(-res);
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
