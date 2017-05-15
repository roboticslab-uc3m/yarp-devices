// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canSetBaudRate(unsigned int rate)
{
    CD_INFO("(%d)\n", rate);

    if (::ioctl(fileDescriptor, IOC_SET_BITRATE, &rate) != 0)
    {
        CD_ERROR("Could not set bitrate.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canGetBaudRate(unsigned int * rate)
{
    CD_INFO("\n");

    if (::ioctl(fileDescriptor, IOC_GET_BITRATE, rate) != 0)
    {
        CD_ERROR("Could not get bitrate.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canIdAdd(unsigned int id)
{
    CD_INFO("(%d)\n", id);

    if (id > 0x7F)
    {
        CD_ERROR("Invalid ID (%d > 0x7F).\n", id);
        return false;
    }

    struct can_filter filter;
    filter.type = FTYPE_AMASK;
    filter.mask = 0x7F;  //-- dsPIC style, mask specifies "do care" bits
    filter.code = id;

    if (::ioctl(fileDescriptor, IOC_SET_FILTER, &filter) != 0)
    {
        CD_ERROR("Could not set filter.\n");
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
    int ret;

    for (unsigned int i = 0; i < size; i++)
    {
        if (wait && i != 0)
        {
            yarp::os::Time::delay(DELAY);
        }

        fd_set fds;
        struct timeval tv;
        FD_ZERO(&fds);

        tv.tv_sec = DELAY;
        tv.tv_usec = 0;

        FD_SET(fileDescriptor, &fds);

        //-- select() returns the number of ready descriptors, or -1 for errors.
        ret = ::select(fileDescriptor + 1, &fds, 0, 0, &tv);

        if (ret <= 0)
        {
            //-- No CD as select() timeout is way too verbose, happens all the time.
            return false;  // Return 0 on select timeout, <0 on select error.
        }

        assert(FD_ISSET(fileDescriptor, &fds));

        yarp::dev::CanMessage & msg = msgs[i];
        struct can_msg * _msg = reinterpret_cast<struct can_msg *>(msg.getPointer());

        canBusReady.wait();
        //-- read() returns the number read, -1 for errors or 0 for EOF.
        ret = ::read(fileDescriptor, _msg, sizeof(struct can_msg));
        canBusReady.post();

        if (ret < 0)
        {
            CD_WARNING("read() error: %s.\n", strerror(errno));
        }
        else
        {
            (*read)++;
        }
    }

    if (*read < size)
    {
        CD_ERROR("read (%d) < size (%d)\n", *read, size);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    int ret;

    for (unsigned int i = 0; i < size; i++)
    {
        if (wait && i != 0)
        {
            yarp::os::Time::delay(DELAY);
        }

        const yarp::dev::CanMessage & msg = const_cast<yarp::dev::CanBuffer &>(msgs)[i];
        const struct can_msg * _msg = reinterpret_cast<const struct can_msg *>(msg.getPointer());

        canBusReady.wait();
        ret = ::write(fileDescriptor, _msg, sizeof(struct can_msg));
        canBusReady.post();

        if (ret == -1)
        {
            CD_WARNING("%s.\n", ::strerror(errno));
        }
        else
        {
            (*sent)++;
        }
    }

    if (*sent < size)
    {
        CD_ERROR("sent (%d) < size (%d)\n", *sent, size);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
