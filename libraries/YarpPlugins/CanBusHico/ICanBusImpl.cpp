// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <cstring>

#include <ColorDebug.hpp>

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canSetBaudRate(unsigned int rate)
{
    CD_DEBUG("(%d)\n", rate);

    int ret;

    canBusReady.wait();
    ret = ::ioctl(fileDescriptor, IOC_SET_BITRATE, &rate);
    canBusReady.post();

    if (ret != 0)
    {
        CD_ERROR("Could not set bitrate: %s.\n", std::strerror(errno));
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canGetBaudRate(unsigned int * rate)
{
    CD_DEBUG("\n");

    int ret;

    canBusReady.wait();
    ret = ::ioctl(fileDescriptor, IOC_GET_BITRATE, rate);
    canBusReady.post();

    if (ret != 0)
    {
        CD_ERROR("Could not set bitrate: %s.\n", std::strerror(errno));
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canIdAdd(unsigned int id)
{
    CD_DEBUG("(%d)\n", id);

    if (id > 0x7F)
    {
        CD_ERROR("Invalid ID (%d > 0x7F).\n", id);
        return false;
    }

    struct can_filter filter;
    filter.type = FTYPE_AMASK;
    filter.mask = 0x7F;  //-- dsPIC style, mask specifies "do care" bits
    filter.code = id;

    int ret;

    canBusReady.wait();
    ret = ::ioctl(fileDescriptor, IOC_SET_FILTER, &filter);
    canBusReady.post();

    if (ret != 0)
    {
        CD_ERROR("Could not set filter: %s.\n", std::strerror(errno));
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canIdDelete(unsigned int id)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    bool ok = true;

    canBusReady.wait();

    if (!setFdMode(wait))
    {
        CD_WARNING("setFdMode() failed: %s.\n", std::strerror(errno));
    }

    for (unsigned int i = 0; i < size; i++)
    {
        if (wait && !setDelay())
        {
            ok = false;
            break;
        }

        yarp::dev::CanMessage & msg = msgs[i];
        struct can_msg * _msg = reinterpret_cast<struct can_msg *>(msg.getPointer());

        //-- read() returns the number read, -1 for errors or 0 for EOF.
        if (::read(fileDescriptor, _msg, sizeof(struct can_msg)) < 0)
        {
            CD_WARNING("read() error: %s.\n", std::strerror(errno));
        }
        else
        {
            (*read)++;
        }
    }

    if (*read < size)
    {
        CD_ERROR("read (%d) < size (%d)\n", *read, size);
        ok = false;
    }

    canBusReady.post();

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    bool ok = true;

    canBusReady.wait();

    if (!setFdMode(wait))
    {
        CD_WARNING("setFdMode() failed: %s.\n", std::strerror(errno));
    }

    for (unsigned int i = 0; i < size; i++)
    {
        // 'wait' param not handled

        const yarp::dev::CanMessage & msg = const_cast<yarp::dev::CanBuffer &>(msgs)[i];
        const struct can_msg * _msg = reinterpret_cast<const struct can_msg *>(msg.getPointer());

        if (::write(fileDescriptor, _msg, sizeof(struct can_msg)) == -1)
        {
            CD_WARNING("%s.\n", std::strerror(errno));
        }
        else
        {
            (*sent)++;
        }
    }

    if (*sent < size)
    {
        CD_ERROR("sent (%d) < size (%d)\n", *sent, size);
        ok = false;
    }

    canBusReady.post();

    return ok;
}

// -----------------------------------------------------------------------------
