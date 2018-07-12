// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>

#include <cstring>
#include <cerrno>
#include <cassert>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

namespace
{
    void setTimeval(int timeMs, struct timeval * tv)
    {
        tv->tv_sec = timeMs / 1000;
        tv->tv_usec = (timeMs % 1000) * 1000;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::setFdMode(bool requestedBlocking)
{
    bool currentlyBlocking = (fcntlFlags & O_NONBLOCK) == 0;

    if (currentlyBlocking != requestedBlocking)
    {
        int flags = fcntlFlags;

        if (requestedBlocking)
        {
            flags &= ~O_NONBLOCK;
        }
        else
        {
            flags |= O_NONBLOCK;
        }

        if (::fcntl(fileDescriptor, F_SETFL, flags) == -1)
        {
            CD_ERROR("fcntl() error: %s.\n", std::strerror(errno));
            return false;
        }

        fcntlFlags = flags;
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
