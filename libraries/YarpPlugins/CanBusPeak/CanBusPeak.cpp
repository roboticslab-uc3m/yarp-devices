// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusPeak.hpp"

#include <sys/select.h>
#include <sys/time.h>

#include <cstring>
#include <cassert>
#include <cerrno>

#include <yarp/conf/version.h>
#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

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

bool CanBusPeak::waitUntilTimeout(io_operation op, bool * bufferReady)
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
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(PEAK, id(), "Unhandled IO operation on select()");
#else
        yCError(PEAK, "Unhandled IO operation on select()");
#endif
        return false;
    }

    if (ret < 0)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(PEAK, id(), "select() error: %s", std::strerror(errno));
#else
        yCError(PEAK, "select() error: %s", std::strerror(errno));
#endif
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

std::uint64_t CanBusPeak::computeAcceptanceCodeAndMask()
{
    // DISCARD message if the following holds true (from driver/pcan_main.c):
    // (pf->id & ~dev->acc_11b.mask) != dev->acc_11b.code

    // From PCAN-Parameter_Documentation.pdf (shipped with PCAN_Basic)
    // Appendix D: Acceptance Code and Mask Calculation

    std::uint32_t mask = ~0x7f & 0x07ff;

    if (activeFilters.empty())
    {
        return (std::uint64_t)mask;
    }

    std::uint32_t code = ~0x0;
    std::uint32_t prevId = 0x0;

    bool firstRun = true;

    for (std::set<unsigned int>::const_iterator it = activeFilters.begin(); it != activeFilters.end(); ++it)
    {
        // logical AND
        code &= *it;

        if (!firstRun)
        {
            // "kind of" logical XOR, only one difference between consecutive IDs
            // is enough to set the "don't care" bit
            mask |= (prevId & ~mask) ^ *it;
        }

        prevId = *it;
        firstRun = false;
    }

    return ((std::uint64_t)code << 32) | mask;
}

// -----------------------------------------------------------------------------
