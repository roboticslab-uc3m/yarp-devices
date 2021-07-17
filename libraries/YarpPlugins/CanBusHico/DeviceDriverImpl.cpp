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

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool CanBusHico::open(yarp::os::Searchable& config)
{
    yCDebug(HICO) << "Config:" << config.toString();

    std::string devicePath = config.check("port", yarp::os::Value(DEFAULT_PORT), "CAN device path").asString();
    int bitrate = config.check("bitrate", yarp::os::Value(DEFAULT_BITRATE), "CAN bitrate (bps)").asInt32();

    blockingMode = config.check("blockingMode", yarp::os::Value(DEFAULT_BLOCKING_MODE), "CAN blocking mode enabled").asBool();
    allowPermissive = config.check("allowPermissive", yarp::os::Value(DEFAULT_ALLOW_PERMISSIVE), "CAN read/write permissive mode").asBool();

    if (blockingMode)
    {
        yCInfo(HICO) << "Blocking mode enabled for CAN device" << devicePath;

        rxTimeoutMs = config.check("rxTimeoutMs", yarp::os::Value(DEFAULT_RX_TIMEOUT_MS), "CAN RX timeout (milliseconds)").asInt32();
        txTimeoutMs = config.check("txTimeoutMs", yarp::os::Value(DEFAULT_TX_TIMEOUT_MS), "CAN TX timeout (milliseconds)").asInt32();

        if (rxTimeoutMs <= 0)
        {
            yCWarning(HICO) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready:" << devicePath;
        }

        if (txTimeoutMs <= 0)
        {
            yCWarning(HICO) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready:" << devicePath;
        }
    }
    else
    {
        yCInfo(HICO) << "Requested non-blocking mode for CAN device" << devicePath;
    }

    yCInfo(HICO, "Permissive mode flag for read/write operations on CAN device %s: %d", devicePath.c_str(), allowPermissive);

    std::string filterConfigStr = config.check("filterConfiguration", yarp::os::Value(DEFAULT_FILTER_CONFIGURATION),
            "CAN filter configuration (disabled|noRange|maskAndRange)").asString();

    yCInfo(HICO, "CAN filter configuration for CAN device %s: %s", devicePath.c_str(), filterConfigStr.c_str());

    filterConfig = FilterManager::parseFilterConfiguration(filterConfigStr);

    //-- Open the CAN device for reading and writing.
    fileDescriptor = ::open(devicePath.c_str(), O_RDWR);

    if (fileDescriptor == -1)
    {
        yCError(HICO) << "Could not open CAN device" << devicePath;
        return false;
    }

    yCInfo(HICO) << "Opened CAN device" << devicePath;

    initBitrateMap();

    //-- Set the CAN bitrate.
    if (!canSetBaudRate(bitrate))
    {
        yCError(HICO) << "Could not set bitrate on CAN device" << devicePath;
        return false;
    }

    yCInfo(HICO) << "Bitrate set on CAN device" << devicePath;

    if (!blockingMode)
    {
        int fcntlFlags = ::fcntl(fileDescriptor, F_GETFL);

        if (fcntlFlags == -1)
        {
            yCError(HICO) << "Unable to retrieve FD flags on CAN device" << devicePath;
            return false;
        }

        fcntlFlags |= O_NONBLOCK;

        if (::fcntl(fileDescriptor, F_SETFL, fcntlFlags) == -1)
        {
            yCError(HICO, "Unable to set non-blocking mode on CAN device %s; fcntl() error: %s", devicePath.c_str(), std::strerror(errno));
            return false;
        }

        yCInfo(HICO) << "Non-blocking mode enabled on CAN device" << devicePath;
    }

    if (filterConfig != FilterManager::DISABLED)
    {
        filterManager = new FilterManager(fileDescriptor, filterConfig == FilterManager::MASK_AND_RANGE);

        if (!config.check("preserveFilters", "don't clear acceptance filters on init"))
        {
            if (!filterManager->clearFilters())
            {
                yCError(HICO) << "Unable to clear acceptance filters on CAN device" << devicePath;
                return false;
            }
            else
            {
                yCInfo(HICO) << "Acceptance filters cleared on CAN device" << devicePath;
            }
        }
        else
        {
            yCWarning(HICO) << "Preserving previous acceptance filters (if any) on CAN device" << devicePath;
        }

        //-- Load initial node IDs and set acceptance filters.
        if (config.check("ids", "initial node IDs"))
        {
            const yarp::os::Bottle & ids = config.findGroup("ids").tail();

            if (ids.size() != 0)
            {
                yCInfo(HICO) << "Parsing bottle of ids on CAN device" << ids.toString();

                if (!filterManager->parseIds(ids))
                {
                    yCError(HICO) << "Could not set acceptance filters on CAN device" << devicePath;
                    return false;
                }

                if (!filterManager->isValid())
                {
                    yCWarning(HICO) << "Hardware limit was hit on CAN device" << devicePath << "and no acceptance filters are enabled";
                }
            }
            else
            {
                yCInfo(HICO) << "No bottle of ids given to CAN device" << devicePath;
            }
        }
    }
    else
    {
        yCInfo(HICO) << "Acceptance filters are disabled for CAN device" << devicePath;
    }

    //-- Start the CAN device.
    if (::ioctl(fileDescriptor,IOC_START) == -1)
    {
        yCError(HICO) << "IOC_START failed on CAN device" << devicePath;
        return false;
    }

    yCInfo(HICO) << "IOC_START ok on CAN device" << devicePath;

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
