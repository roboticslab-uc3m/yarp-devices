// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusPeak.hpp"

#include <cstring> // std::strerror

#include <yarp/os/LogStream.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusPeak::canGetErrors(yarp::dev::CanErrors & err)
{
    struct pcanfd_state pfds;

    {
        std::lock_guard<std::mutex> lockGuard(canBusReady);
        int res = pcanfd_get_state(fileDescriptor, &pfds);

        if (res < 0)
        {
            yError() << "pcanfd_get_state() failed:" << std::strerror(-res);
            return false;
        }
    }

    err.rxCanErrors = pfds.rx_error_counter;
    err.txCanErrors = pfds.tx_error_counter;

    err.rxBufferOvr = pfds.rx_pending_msgs;
    err.txBufferOvr = pfds.tx_pending_msgs;

    err.rxCanFifoOvr = 0;
    err.txCanFifoOvr = 0;

    err.busoff = pfds.bus_state == PCANFD_ERROR_BUSOFF;

    return true;
}

// -----------------------------------------------------------------------------
