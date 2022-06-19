// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <cstring> // std::strerror
#include <cerrno>

#include <string>

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
    std::string devicePath = config.check("port", yarp::os::Value(DEFAULT_PORT), "CAN device path").asString();
    int bitrate = config.check("bitrate", yarp::os::Value(DEFAULT_BITRATE), "CAN bitrate (bps)").asInt32();

    yarp::dev::DeviceDriver::setId(devicePath);

    blockingMode = config.check("blockingMode", yarp::os::Value(DEFAULT_BLOCKING_MODE), "CAN blocking mode enabled").asBool();
    allowPermissive = config.check("allowPermissive", yarp::os::Value(DEFAULT_ALLOW_PERMISSIVE), "CAN read/write permissive mode").asBool();

    if (blockingMode)
    {
        yCIInfo(HICO, id()) << "Blocking mode enabled";

        rxTimeoutMs = config.check("rxTimeoutMs", yarp::os::Value(DEFAULT_RX_TIMEOUT_MS), "CAN RX timeout (milliseconds)").asInt32();
        txTimeoutMs = config.check("txTimeoutMs", yarp::os::Value(DEFAULT_TX_TIMEOUT_MS), "CAN TX timeout (milliseconds)").asInt32();

        if (rxTimeoutMs <= 0)
        {
            yCIWarning(HICO, id()) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready";
        }

        if (txTimeoutMs <= 0)
        {
            yCIWarning(HICO, id()) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready";
        }
    }
    else
    {
        yCIInfo(HICO, id()) << "Requested non-blocking mode";
    }

    yCIInfo(HICO, id()) << "Permissive mode flag for read/write operations:" << allowPermissive;

    std::string filterConfigStr = config.check("filterConfiguration", yarp::os::Value(DEFAULT_FILTER_CONFIGURATION),
            "CAN filter configuration (disabled|noRange|maskAndRange)").asString();

    yCIInfo(HICO, id()) << "CAN filter configuration:" << filterConfigStr;

    filterConfig = parseFilterConfiguration(filterConfigStr);

    //-- Open the CAN device for reading and writing.
    fileDescriptor = ::open(devicePath.c_str(), O_RDWR);

    if (fileDescriptor == -1)
    {
        yCIError(HICO, id()) << "Could not open";
        return false;
    }

    yCIInfo(HICO, id()) << "Successfully opened";

    //-- Set the CAN bitrate.
    if (!canSetBaudRate(bitrate))
    {
        yCIError(HICO, id()) << "Could not set bitrate";
        return false;
    }

    yCIInfo(HICO, id()) << "Bitrate set";

    if (!blockingMode)
    {
        int fcntlFlags = ::fcntl(fileDescriptor, F_GETFL);

        if (fcntlFlags == -1)
        {
            yCIError(HICO, id()) << "Unable to retrieve FD flags";
            return false;
        }

        fcntlFlags |= O_NONBLOCK;

        if (::fcntl(fileDescriptor, F_SETFL, fcntlFlags) == -1)
        {
            yCIError(HICO, id()) << "Unable to set non-blocking mode:" << std::strerror(errno);
            return false;
        }

        yCIInfo(HICO, id()) << "Non-blocking mode enabled";
    }

    if (filterConfig != FilterManager::DISABLED)
    {
        filterManager = new FilterManager(*this, fileDescriptor, filterConfig == FilterManager::MASK_AND_RANGE);

        if (!config.check("preserveFilters", "don't clear acceptance filters on init"))
        {
            if (!filterManager->clearFilters())
            {
                yCIError(HICO, id()) << "Unable to clear acceptance filters";
                return false;
            }
            else
            {
                yCIInfo(HICO, id()) << "Acceptance filters cleared";
            }
        }
        else
        {
            yCIWarning(HICO, id()) << "Preserving previous acceptance filters (if any)";
        }

        //-- Load initial node IDs and set acceptance filters.
        if (config.check("filteredIds", "filtered node IDs"))
        {
            const yarp::os::Bottle * ids = config.findGroup("filteredIds").get(1).asList();

            if (ids->size() != 0)
            {
                yCIInfo(HICO, id()) << "Parsing bottle of ids";

                if (!filterManager->parseIds(*ids))
                {
                    yCIError(HICO, id()) << "Could not set acceptance filters";
                    return false;
                }

                if (!filterManager->isValid())
                {
                    yCIWarning(HICO, id()) << "Hardware limit was hit and no acceptance filters are enabled";
                }
            }
            else
            {
                yCIInfo(HICO, id()) << "No bottle of ids provided";
            }
        }
    }
    else
    {
        yCIInfo(HICO, id()) << "Acceptance filters are disabled";
    }

    //-- Start the CAN device.
    if (::ioctl(fileDescriptor,IOC_START) == -1)
    {
        yCIError(HICO, id()) << "IOC_START failed";
        return false;
    }

    yCIInfo(HICO, id()) << "IOC_START ok";

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::close()
{
    if (fileDescriptor > 0)
    {
        if (filterConfig != FilterManager::DISABLED && filterManager)
        {
            filterManager->clearFilters();
            delete filterManager;
            filterManager = nullptr;
        }

        ::close(fileDescriptor);
        fileDescriptor = 0;
    }

    return true;
}

// -----------------------------------------------------------------------------
