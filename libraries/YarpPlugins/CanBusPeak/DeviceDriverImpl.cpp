// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusPeak.hpp"

#include <cstring> // std::strerror

#include <string>

#include <ColorDebug.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CanBusPeak::open(yarp::os::Searchable& config)
{
    std::string devicePath = config.check("canDevice", yarp::os::Value(DEFAULT_CAN_DEVICE), "CAN device path").asString();

    int bitrate = config.check("canBitrate", yarp::os::Value(DEFAULT_CAN_BITRATE), "CAN bitrate").asInt();

    nonBlockingMode = config.check("canNonBlockingMode", "CAN non-blocking mode enabled");

    int flags = OFD_BITRATE;

    if (!nonBlockingMode)
    {
        rxTimeoutMs = config.check("canRxTimeoutMs", yarp::os::Value(DEFAULT_CAN_RX_TIMEOUT_MS), "RX timeout (milliseconds)").asInt();
        txTimeoutMs = config.check("canTxTimeoutMs", yarp::os::Value(DEFAULT_CAN_TX_TIMEOUT_MS), "TX timeout (milliseconds)").asInt();

        if (rxTimeoutMs <= 0)
        {
            CD_WARNING("RX timeout value <= 0, CAN read calls will block until the buffer is ready.\n");
        }

        if (txTimeoutMs <= 0)
        {
            CD_WARNING("TX timeout value <= 0, CAN write calls will block until the buffer is ready.\n");
        }
    }
    else
    {
        CD_INFO("Non-blocking mode enabled.\n");
        flags |= OFD_NONBLOCKING;
    }

    int res = pcanfd_open(devicePath.c_str(), flags, bitrate);

    if (res < 0)
    {
        CD_ERROR("Could not open CAN device of path: %s.\n", devicePath.c_str(), std::strerror(-res));
        return false;
    }
    else
    {
        CD_SUCCESS("Successfully opened CAN device of path: %s.\n", devicePath.c_str());
        fileDescriptor = res;
    }

    if (!config.check("preserveFilters", "don't clear acceptance filters on init"))
    {
        res = pcanfd_del_filters(fileDescriptor);

        if (res < 0)
        {
            CD_ERROR("Unable to clear acceptance filters on CAN device: %s (%s).\n", devicePath.c_str(), std::strerror(-res));
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

            for (int i = 0; i < ids.size(); i++)
            {
                activeFilters.insert(ids.get(i).asDouble());
            }

            uint64_t acc = computeAcceptanceCodeAndMask();

            CD_DEBUG("New acceptance code+mask: %016lxh.\n", acc);

            res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

            if (res < 0)
            {
                CD_ERROR("Unable to set acceptance filters on CAN device: %s (%s)\n", devicePath.c_str(), std::strerror(-res));
                activeFilters.clear();
                return false;
            }

            CD_SUCCESS("Initial IDs added to set of acceptance filters: %s.\n", devicePath.c_str());
        }
        else
        {
            CD_INFO("No bottle of ids given to CAN device.\n");
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::close()
{
    if (fileDescriptor > 0)
    {
        if (!activeFilters.empty())
        {
            int res = pcanfd_del_filters(fileDescriptor);

            if (res < 0)
            {
                CD_WARNING("Unable to clear acceptance filters (%s).\n", std::strerror(-res));
            }
            else
            {
                activeFilters.clear();
            }
        }

        pcanfd_close(fileDescriptor);
    }

    return true;
}

// -----------------------------------------------------------------------------
