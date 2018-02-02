// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CanBusHico::open(yarp::os::Searchable& config)
{

    std::string devicePath = config.check("canDevice",yarp::os::Value(DEFAULT_CAN_DEVICE),"CAN device path").asString();
    int bitrate = config.check("canBitrate",yarp::os::Value(DEFAULT_CAN_BITRATE),"CAN bitrate").asInt();

    //-- Open the CAN device for reading and writing.
    fileDescriptor = ::open(devicePath.c_str(), O_RDWR);
    if(fileDescriptor<0)
    {
        CD_ERROR("Could not open CAN device of path: %s\n", devicePath.c_str());
        return false;
    }
    CD_SUCCESS("Opened CAN device of path: %s\n", devicePath.c_str());

    yarp::os::Time::delay(DELAY);

    //-- Set the CAN bitrate.
    if( ioctl(fileDescriptor,IOC_SET_BITRATE,&bitrate) != 0)
    {
        CD_ERROR("Could not set bitrate on CAN device: %s\n", devicePath.c_str());
        return false;
    }
    CD_SUCCESS("Bitrate set on CAN device: %s\n", devicePath.c_str());

    yarp::os::Time::delay(DELAY);

    //-- Start the CAN device.
    if( ioctl(fileDescriptor,IOC_START) != 0)
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

    //release semaphore?
    ::close(fileDescriptor);

    return true;
}

// -----------------------------------------------------------------------------

