// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <string>

#include <yarp/os/Time.h>

#include <ColorDebug.hpp>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CanBusHico::open(yarp::os::Searchable& config)
{
    std::string devicePath = config.check("canDevice", yarp::os::Value(DEFAULT_CAN_DEVICE), "CAN device path").asString();
    int bitrate = config.check("canBitrate", yarp::os::Value(DEFAULT_CAN_BITRATE), "CAN bitrate").asInt();

    rxTimeoutMs = config.check("canRxTimeoutMs", yarp::os::Value(DEFAULT_CAN_RX_TIMEOUT_MS), "RX timeout [ms]").asInt();
    txTimeoutMs = config.check("canTxTimeoutMs", yarp::os::Value(DEFAULT_CAN_TX_TIMEOUT_MS), "TX timeout [ms]").asInt();

    if (rxTimeoutMs <= 0)
    {
        CD_WARNING("RX timeout value <= 0, CAN read calls will block until the buffer is ready.\n");
    }

    if (txTimeoutMs <= 0)
    {
        CD_WARNING("TX timeout value <= 0, CAN write calls will block until the buffer is ready.\n");
    }

    //-- Open the CAN device for reading and writing.
    fileDescriptor = ::open(devicePath.c_str(), O_RDWR);

    if (fileDescriptor == -1)
    {
        CD_ERROR("Could not open CAN device of path: %s\n", devicePath.c_str());
        return false;
    }

    CD_SUCCESS("Opened CAN device of path: %s\n", devicePath.c_str());

    yarp::os::Time::delay(DELAY);

    fcntlFlags = ::fcntl(fileDescriptor, F_GETFL);

    if (fcntlFlags == -1)
    {
        CD_ERROR("Could not retrieve FD flags\n");
        return false;
    }

    yarp::os::Time::delay(DELAY);

    //-- Set the CAN bitrate.
    if (!canSetBaudRate(bitrate))
    {
        CD_ERROR("Could not set bitrate on CAN device: %s\n", devicePath.c_str());
        return false;
    }

    CD_SUCCESS("Bitrate set on CAN device: %s\n", devicePath.c_str());

    yarp::os::Time::delay(DELAY);

    //-- Clear acceptance filters
    if (!clearFilters())
    {
        CD_ERROR("Could not clear acceptance filters on CAN device: %s\n", devicePath.c_str());
        return false;
    }

    CD_SUCCESS("Acceptance filters cleared on CAN device: %s\n", devicePath.c_str());

    yarp::os::Time::delay(DELAY);

    //-- Start the CAN device.
    if (::ioctl(fileDescriptor,IOC_START) == -1)
    {
        CD_ERROR("IOC_START failed on CAN device: %s\n", devicePath.c_str());
        return false;
    }

    CD_SUCCESS("IOC_START ok on CAN device: %s\n", devicePath.c_str());

    yarp::os::Time::delay(DELAY);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::close()
{
    if (fileDescriptor > 0)
    {
        clearFilters();
        ::close(fileDescriptor);
    }

    return true;
}

// -----------------------------------------------------------------------------
