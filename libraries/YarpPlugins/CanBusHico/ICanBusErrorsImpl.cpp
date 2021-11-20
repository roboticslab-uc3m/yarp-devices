// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <cstring>
#include <cerrno>

#include <yarp/conf/version.h>
#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusHico::canGetErrors(yarp::dev::CanErrors & err)
{
    std::lock_guard<std::mutex> lockGuard(canBusReady);

    unsigned int status;

    if (::ioctl(fileDescriptor, IOC_GET_CAN_STATUS, &status) == -1)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id(), "Could not query CAN status: %s", std::strerror(errno));
#else
        yCError(HICO, "Could not query CAN status: %s", std::strerror(errno));
#endif
        return false;
    }

    err.rxCanErrors = CS_GET_RXERRCNT(status);
    err.txCanErrors = CS_GET_TXERRCNT(status);

    err.rxBufferOvr = 0;
    err.txBufferOvr = 0;

    err.rxCanFifoOvr = 0;
    err.txCanFifoOvr = 0;

    err.busoff = status & (CS_ERROR_PASSIVE | CS_ERROR_BUS_OFF);

    return true;
}

// -----------------------------------------------------------------------------
