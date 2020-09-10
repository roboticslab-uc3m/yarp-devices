// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <cstring> // std::strerror
#include <cerrno>

#include <string>

#include <yarp/os/Time.h>

#include <ColorDebug.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CanBusHico::open(yarp::os::Searchable& config)
{
    CD_DEBUG("%s\n", config.toString().c_str());

    std::string devicePath = config.check("port", yarp::os::Value(DEFAULT_PORT), "CAN device path").asString();
    int bitrate = config.check("bitrate", yarp::os::Value(DEFAULT_BITRATE), "CAN bitrate (bps)").asInt32();

    blockingMode = config.check("blockingMode", yarp::os::Value(DEFAULT_BLOCKING_MODE), "CAN blocking mode enabled").asBool();
    allowPermissive = config.check("allowPermissive", yarp::os::Value(DEFAULT_ALLOW_PERMISSIVE), "CAN read/write permissive mode").asBool();

    if (blockingMode)
    {
        CD_INFO("Blocking mode enabled for CAN device: %s.\n", devicePath.c_str());

        rxTimeoutMs = config.check("rxTimeoutMs", yarp::os::Value(DEFAULT_RX_TIMEOUT_MS), "CAN RX timeout (milliseconds)").asInt32();
        txTimeoutMs = config.check("txTimeoutMs", yarp::os::Value(DEFAULT_TX_TIMEOUT_MS), "CAN TX timeout (milliseconds)").asInt32();

        if (rxTimeoutMs <= 0)
        {
            CD_WARNING("RX timeout value <= 0, CAN read calls will block until the buffer is ready: %s.\n", devicePath.c_str());
        }

        if (txTimeoutMs <= 0)
        {
            CD_WARNING("TX timeout value <= 0, CAN write calls will block until the buffer is ready: %s.\n", devicePath.c_str());
        }
    }
    else
    {
        CD_INFO("Requested non-blocking mode for CAN device: %s.\n", devicePath.c_str());
    }

    CD_INFO("Permissive mode flag for read/write operations on CAN device %s: %d.\n", devicePath.c_str(), allowPermissive);

    std::string filterConfigStr = config.check("filterConfiguration", yarp::os::Value(DEFAULT_FILTER_CONFIGURATION),
            "CAN filter configuration (disabled|noRange|maskAndRange)").asString();

    CD_INFO("CAN filter configuration for CAN device %s: %s.\n", devicePath.c_str(), filterConfigStr.c_str());

    filterConfig = FilterManager::parseFilterConfiguration(filterConfigStr);

    //-- Open the CAN device for reading and writing.
    fileDescriptor = ::open(devicePath.c_str(), O_RDWR);

    if (fileDescriptor == -1)
    {
        CD_ERROR("Could not open CAN device of path: %s\n", devicePath.c_str());
        return false;
    }

    CD_SUCCESS("Opened CAN device of path: %s\n", devicePath.c_str());

    initBitrateMap();

    //-- Set the CAN bitrate.
    if (!canSetBaudRate(bitrate))
    {
        CD_ERROR("Could not set bitrate on CAN device: %s.\n", devicePath.c_str());
        return false;
    }

    CD_SUCCESS("Bitrate set on CAN device: %s.\n", devicePath.c_str());

    if (!blockingMode)
    {
        int fcntlFlags = ::fcntl(fileDescriptor, F_GETFL);

        if (fcntlFlags == -1)
        {
            CD_ERROR("Unable to retrieve FD flags on CAN device %s.\n", devicePath.c_str());
            return false;
        }

        fcntlFlags |= O_NONBLOCK;

        if (::fcntl(fileDescriptor, F_SETFL, fcntlFlags) == -1)
        {
            CD_ERROR("Unable to set non-blocking mode on CAN device %s; fcntl() error: %s.\n", devicePath.c_str(), std::strerror(errno));
            return false;
        }

        CD_SUCCESS("Non-blocking mode enabled on CAN device: %s.\n", devicePath.c_str());
    }

    if (filterConfig != FilterManager::DISABLED)
    {
        filterManager = new FilterManager(fileDescriptor, filterConfig == FilterManager::MASK_AND_RANGE);

        if (!config.check("preserveFilters", "don't clear acceptance filters on init"))
        {
            if (!filterManager->clearFilters())
            {
                CD_ERROR("Unable to clear acceptance filters on CAN device: %s.\n", devicePath.c_str());
                return false;
            }
            else
            {
                CD_SUCCESS("Acceptance filters cleared on CAN device: %s.\n", devicePath.c_str());
            }
        }
        else
        {
            CD_WARNING("Preserving previous acceptance filters (if any): %s.\n", devicePath.c_str());
        }

        //-- Load initial node IDs and set acceptance filters.
        if (config.check("ids", "initial node IDs"))
        {
            const yarp::os::Bottle & ids = config.findGroup("ids").tail();

            if (ids.size() != 0)
            {
                CD_INFO("Parsing bottle of ids on CAN device: %s.\n", ids.toString().c_str());

                if (!filterManager->parseIds(ids))
                {
                    CD_ERROR("Could not set acceptance filters on CAN device: %s\n", devicePath.c_str());
                    return false;
                }

                if (!filterManager->isValid())
                {
                    CD_WARNING("Hardware limit was hit on CAN device %s, no acceptance filters are enabled.\n",
                            devicePath.c_str());
                }
            }
            else
            {
                CD_INFO("No bottle of ids given to CAN device: %s.\n", devicePath.c_str());
            }
        }
    }
    else
    {
        CD_INFO("Acceptance filters are disabled for CAN device: %s.\n", devicePath.c_str());
    }

    //-- Start the CAN device.
    if (::ioctl(fileDescriptor,IOC_START) == -1)
    {
        CD_ERROR("IOC_START failed on CAN device: %s.\n", devicePath.c_str());
        return false;
    }

    CD_SUCCESS("IOC_START ok on CAN device: %s.\n", devicePath.c_str());

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::close()
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
