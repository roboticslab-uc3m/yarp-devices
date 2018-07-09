// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusPeak.hpp"

#include <cstring> // std::strerror

#include <string>

#include <ColorDebug.hpp>

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

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::close()
{
    pcanfd_close(fileDescriptor);
    return true;
}

// -----------------------------------------------------------------------------
