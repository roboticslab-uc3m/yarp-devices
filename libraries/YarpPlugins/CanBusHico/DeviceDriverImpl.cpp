// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <cstring> // std::strerror
#include <cerrno>

#include <string>

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

constexpr auto DEFAULT_PORT = "/dev/can0";
constexpr auto DEFAULT_BITRATE = 1000000;

constexpr auto DEFAULT_RX_TIMEOUT_MS = 1;
constexpr auto DEFAULT_TX_TIMEOUT_MS = 0; // '0' means no timeout

constexpr auto DEFAULT_BLOCKING_MODE = true;
constexpr auto DEFAULT_ALLOW_PERMISSIVE = false;

constexpr auto DELAY = 0.001; // [s]

constexpr auto DEFAULT_FILTER_CONFIGURATION = "disabled";

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool CanBusHico::open(yarp::os::Searchable& config)
{
#if !defined(YARP_VERSION_COMPARE) // < 3.6.0
    yCDebug(HICO) << "Config:" << config.toString();
#endif

    std::string devicePath = config.check("port", yarp::os::Value(DEFAULT_PORT), "CAN device path").asString();
    int bitrate = config.check("bitrate", yarp::os::Value(DEFAULT_BITRATE), "CAN bitrate (bps)").asInt32();

#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yarp::dev::DeviceDriver::setId(devicePath);
#endif

    blockingMode = config.check("blockingMode", yarp::os::Value(DEFAULT_BLOCKING_MODE), "CAN blocking mode enabled").asBool();
    allowPermissive = config.check("allowPermissive", yarp::os::Value(DEFAULT_ALLOW_PERMISSIVE), "CAN read/write permissive mode").asBool();

    if (blockingMode)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIInfo(HICO, id()) << "Blocking mode enabled";
#else
        yCInfo(HICO) << "Blocking mode enabled for CAN device" << devicePath;
#endif

        rxTimeoutMs = config.check("rxTimeoutMs", yarp::os::Value(DEFAULT_RX_TIMEOUT_MS), "CAN RX timeout (milliseconds)").asInt32();
        txTimeoutMs = config.check("txTimeoutMs", yarp::os::Value(DEFAULT_TX_TIMEOUT_MS), "CAN TX timeout (milliseconds)").asInt32();

        if (rxTimeoutMs <= 0)
        {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
            yCIWarning(HICO, id()) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready";
#else
            yCWarning(HICO) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready:" << devicePath;
#endif
        }

        if (txTimeoutMs <= 0)
        {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
            yCIWarning(HICO, id()) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready";
#else
            yCWarning(HICO) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready:" << devicePath;
#endif
        }
    }
    else
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIInfo(HICO, id()) << "Requested non-blocking mode";
#else
        yCInfo(HICO) << "Requested non-blocking mode for CAN device" << devicePath;
#endif
    }

#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCIInfo(HICO, id()) << "Permissive mode flag for read/write operations:" << allowPermissive;
#else
    yCInfo(HICO, "Permissive mode flag for read/write operations on CAN device %s: %d", devicePath.c_str(), allowPermissive);
#endif

    std::string filterConfigStr = config.check("filterConfiguration", yarp::os::Value(DEFAULT_FILTER_CONFIGURATION),
            "CAN filter configuration (disabled|noRange|maskAndRange)").asString();

#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCIInfo(HICO, id()) << "CAN filter configuration:" << filterConfigStr;
#else
    yCInfo(HICO, "CAN filter configuration for CAN device %s: %s", devicePath.c_str(), filterConfigStr.c_str());
#endif

    filterConfig = parseFilterConfiguration(filterConfigStr);

    //-- Open the CAN device for reading and writing.
    fileDescriptor = ::open(devicePath.c_str(), O_RDWR);

    if (fileDescriptor == -1)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id()) << "Could not open";
#else
        yCError(HICO) << "Could not open CAN device" << devicePath;
#endif
        return false;
    }

#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCIInfo(HICO, id()) << "Successfully opened";
#else
    yCInfo(HICO) << "Opened CAN device" << devicePath;
#endif

    initBitrateMap();

    //-- Set the CAN bitrate.
    if (!canSetBaudRate(bitrate))
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id()) << "Could not set bitrate";
#else
        yCError(HICO) << "Could not set bitrate on CAN device" << devicePath;
#endif
        return false;
    }

#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCIInfo(HICO, id()) << "Bitrate set";
#else
    yCInfo(HICO) << "Bitrate set on CAN device" << devicePath;
#endif

    if (!blockingMode)
    {
        int fcntlFlags = ::fcntl(fileDescriptor, F_GETFL);

        if (fcntlFlags == -1)
        {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
            yCIError(HICO, id()) << "Unable to retrieve FD flags";
#else
            yCError(HICO) << "Unable to retrieve FD flags on CAN device" << devicePath;
#endif
            return false;
        }

        fcntlFlags |= O_NONBLOCK;

        if (::fcntl(fileDescriptor, F_SETFL, fcntlFlags) == -1)
        {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
            yCIError(HICO, id()) << "Unable to set non-blocking mode:" << std::strerror(errno);
#else
            yCError(HICO, "Unable to set non-blocking mode on CAN device %s; fcntl() error: %s", devicePath.c_str(), std::strerror(errno));
#endif
            return false;
        }

#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIInfo(HICO, id()) << "Non-blocking mode enabled";
#else
        yCInfo(HICO) << "Non-blocking mode enabled on CAN device" << devicePath;
#endif
    }

    if (filterConfig != FilterManager::DISABLED)
    {
        filterManager = new FilterManager(*this, fileDescriptor, filterConfig == FilterManager::MASK_AND_RANGE);

        if (!config.check("preserveFilters", "don't clear acceptance filters on init"))
        {
            if (!filterManager->clearFilters())
            {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
                yCIError(HICO, id()) << "Unable to clear acceptance filters";
#else
                yCError(HICO) << "Unable to clear acceptance filters on CAN device" << devicePath;
#endif
                return false;
            }
            else
            {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
                yCIInfo(HICO, id()) << "Acceptance filters cleared";
#else
                yCInfo(HICO) << "Acceptance filters cleared on CAN device" << devicePath;
#endif
            }
        }
        else
        {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
            yCIWarning(HICO, id()) << "Preserving previous acceptance filters (if any)";
#else
            yCWarning(HICO) << "Preserving previous acceptance filters (if any) on CAN device" << devicePath;
#endif
        }

        //-- Load initial node IDs and set acceptance filters.
        if (config.check("filteredIds", "filtered node IDs"))
        {
            const yarp::os::Bottle * ids = config.findGroup("filteredIds").get(1).asList();

            if (ids->size() != 0)
            {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
                yCIInfo(HICO, id()) << "Parsing bottle of ids";
#else
                yCInfo(HICO) << "Parsing bottle of ids on CAN device" << devicePath;
#endif

                if (!filterManager->parseIds(*ids))
                {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
                    yCIError(HICO, id()) << "Could not set acceptance filters";
#else
                    yCError(HICO) << "Could not set acceptance filters on CAN device" << devicePath;
#endif
                    return false;
                }

                if (!filterManager->isValid())
                {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
                    yCIWarning(HICO, id()) << "Hardware limit was hit and no acceptance filters are enabled";
#else
                    yCWarning(HICO) << "Hardware limit was hit on CAN device" << devicePath << "and no acceptance filters are enabled";
#endif
                }
            }
            else
            {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
                yCIInfo(HICO, id()) << "No bottle of ids provided";
#else
                yCInfo(HICO) << "No bottle of ids given to CAN device" << devicePath;
#endif
            }
        }
    }
    else
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIInfo(HICO, id()) << "Acceptance filters are disabled";
#else
        yCInfo(HICO) << "Acceptance filters are disabled for CAN device" << devicePath;
#endif
    }

    //-- Start the CAN device.
    if (::ioctl(fileDescriptor,IOC_START) == -1)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id()) << "IOC_START failed";
#else
        yCError(HICO) << "IOC_START failed on CAN device" << devicePath;
#endif
        return false;
    }

#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yCIInfo(HICO, id()) << "IOC_START ok";
#else
    yCInfo(HICO) << "IOC_START ok on CAN device" << devicePath;
#endif

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::close()
{
    if (fileDescriptor > 0)
    {
        if (filterConfig != FilterManager::DISABLED && filterManager != NULL)
        {
            filterManager->clearFilters();
            delete filterManager;
            filterManager = NULL;
        }

        ::close(fileDescriptor);
        fileDescriptor = 0;
    }

    return true;
}

// -----------------------------------------------------------------------------
