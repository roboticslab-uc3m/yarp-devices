// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <assert.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>

#include <cstring>

#include <ColorDebug.hpp>

// -----------------------------------------------------------------------------

namespace
{
    void setTimeval(int timeMs, struct timeval * tv)
    {
        int sec = timeMs / 1000;
        tv->tv_sec = sec;
        tv->tv_usec = (timeMs - (sec * 1000)) * 1000;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::setFdMode(bool requestedBlocking)
{
    bool currentlyBlocking = (fcntlFlags & O_NONBLOCK) != O_NONBLOCK;

    if (currentlyBlocking != requestedBlocking)
    {
        int flag = requestedBlocking ? ~O_NONBLOCK : O_NONBLOCK;

        if (::fcntl(fileDescriptor, F_SETFL, fcntlFlags & flag) == -1)
        {
            CD_ERROR("fcntl() error: %s.\n", std::strerror(errno));
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::waitUntilTimeout(io_operation op, bool * bufferReady)
{
    fd_set fds;

    FD_ZERO(&fds);
    FD_SET(fileDescriptor, &fds);

    struct timeval tv;

    //-- select() returns the number of ready descriptors, 0 for timeout, -1 for errors.
    int ret;

    switch (op)
    {
    case READ:
        setTimeval(rxTimeoutMs, &tv);
        ret = ::select(fileDescriptor + 1, &fds, 0, 0, &tv);
        break;
    case WRITE:
        setTimeval(txTimeoutMs, &tv);
        ret = ::select(fileDescriptor + 1, 0, &fds, 0, &tv);
        break;
    default:
        CD_ERROR("Unhandled IO operation on select().\n");
        return false;
    }

    if (ret < 0)
    {
        CD_ERROR("select() error: %s.\n", std::strerror(errno));
        return false;
    }
    else if (ret == 0)
    {
        *bufferReady = false;
    }
    else
    {
        assert(FD_ISSET(fileDescriptor, &fds));
        *bufferReady = true;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::clearFilters()
{
    if (::ioctl(fileDescriptor, IOC_CLEAR_FILTERS) == -1)
    {
        CD_ERROR("ioctl() error: %s\n", std::strerror(errno));
        return false;
    }

    filteredIds.clear();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::interpretBitrate(unsigned int rate, std::string & str)
{
    switch (rate)
    {
    case BITRATE_10k:
        str = "10k";
        break;
    case BITRATE_20k:
        str = "20k";
        break;
    case BITRATE_50k:
        str = "50k";
        break;
    case BITRATE_100k:
        str = "100k";
        break;
    case BITRATE_125k:
        str = "125k";
        break;
    case BITRATE_250k:
        str = "250k";
        break;
    case BITRATE_500k:
        str = "500k";
        break;
    case BITRATE_800k:
        str = "800k";
        break;
    case BITRATE_1000k:
        str = "1000k";
        break;
    default:
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
