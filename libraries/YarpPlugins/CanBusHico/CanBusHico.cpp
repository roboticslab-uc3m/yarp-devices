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

bool roboticslab::CanBusHico::setFdMode(bool requestedBlocking)
{
    bool currentlyBlocking = (fcntlFlags & O_NONBLOCK) != O_NONBLOCK;

    if (currentlyBlocking != requestedBlocking)
    {
        int flag = requestedBlocking ? ~O_NONBLOCK : O_NONBLOCK;
        return ::fcntl(fileDescriptor, F_SETFL, fcntlFlags & flag) != -1;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::setDelay()
{
    fd_set fds;
    struct timeval tv;

    tv.tv_sec = DELAY * 1000;
    tv.tv_usec = 0;

    FD_ZERO(&fds);
    FD_SET(fileDescriptor, &fds);

    //-- select() returns the number of ready descriptors, or -1 for errors.
    int ret = ::select(fileDescriptor + 1, &fds, 0, 0, &tv);

    if (ret <= 0)
    {
        //-- No CD as select() timeout is way too verbose, happens all the time.
        // Return 0 on select timeout, <0 on select error.
        return false;
    }

    assert(FD_ISSET(fileDescriptor, &fds));

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::clearFilters()
{
    if (::ioctl(fileDescriptor, IOC_CLEAR_FILTERS) != 0)
    {
        CD_ERROR("Could not clear filters: %s\n", std::strerror(errno));
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
