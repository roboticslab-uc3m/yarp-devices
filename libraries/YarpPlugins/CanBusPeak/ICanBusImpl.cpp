// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusPeak.hpp"

#include <cstring> // std::memset, std::memcpy, std::strerror
#include <cerrno> // error codes

#include <libpcanfd.h>

#include <yarp/conf/version.h>
#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusPeak::canSetBaudRate(unsigned int rate)
{
    struct pcanfd_init pfdi;
    std::memset(&pfdi, '\0', sizeof(pfdi));
    pfdi.nominal.bitrate = rate;

    canBusReady.lock();
    int res = pcanfd_set_init(fileDescriptor, &pfdi);
    canBusReady.unlock();

    if (res < 0)
    {
#if YARP_VERSION_MINOR >= 6
        yCIError(PEAK, id()) << "Unable to set bitrate:" << std::strerror(-res);
#else
        yCError(PEAK) << "Unable to set bitrate:" << std::strerror(-res);
#endif
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusPeak::canGetBaudRate(unsigned int * rate)
{
    struct pcanfd_init pfdi;

    canBusReady.lock();
    int res = pcanfd_get_init(fileDescriptor, &pfdi);
    canBusReady.unlock();

    if (res < 0)
    {
#if YARP_VERSION_MINOR >= 6
        yCIError(PEAK, id()) << "Unable to retrieve bitrate:" << std::strerror(-res);
#else
        yCError(PEAK) << "Unable to retrieve bitrate:" << std::strerror(-res);
#endif
        return false;
    }

    *rate = pfdi.nominal.bitrate;

    if (pfdi.nominal.bitrate != pfdi.nominal.bitrate_real)
    {
#if YARP_VERSION_MINOR >= 6
        yCIWarning(PEAK, id()) << "User-defined nominal bitrate" << pfdi.nominal.bitrate << "differs from real nominal bitrate" << pfdi.nominal.bitrate_real;
#else
        yCWarning(PEAK) << "User-defined nominal bitrate" << pfdi.nominal.bitrate << "differs from real nominal bitrate" << pfdi.nominal.bitrate_real;
#endif
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusPeak::canIdAdd(unsigned int _id)
{
    std::lock_guard<std::mutex> lockGuard(canBusReady);

    if (activeFilters.find(_id) != activeFilters.end())
    {
#if YARP_VERSION_MINOR >= 6
        yCIWarning(PEAK, id()) << "Filter for id" << _id << "already set";
#else
        yCWarning(PEAK) << "Filter for id" << _id << "already set";
#endif
        return true;
    }

    activeFilters.insert(_id);

    std::uint64_t acc = computeAcceptanceCodeAndMask();

#if YARP_VERSION_MINOR >= 6
    yCIDebug(PEAK, id(), "New acceptance code+mask: %016lxh", acc);
#else
    yCDebug(PEAK, "New acceptance code+mask: %016lxh", acc);
#endif

    int res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

    if (res < 0)
    {
#if YARP_VERSION_MINOR >= 6
        yCIError(PEAK, id()) << "pcanfd_set_option() failed:" << std::strerror(-res);
#else
        yCError(PEAK) << "pcanfd_set_option() failed:" << std::strerror(-res);
#endif
        activeFilters.erase(_id);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusPeak::canIdDelete(unsigned int _id)
{
    std::lock_guard<std::mutex> lockGuard(canBusReady);

    if (_id == 0)
    {
#if YARP_VERSION_MINOR >= 6
        yCIInfo(PEAK, id()) << "Clearing filters previously set";
#else
        yCInfo(PEAK) << "Clearing filters previously set";
#endif

        int res = pcanfd_del_filters(fileDescriptor);

        if (res < 0)
        {
#if YARP_VERSION_MINOR >= 6
            yCIError(PEAK, id()) << "Unable to clear accceptance filters:" << std::strerror(-res);
#else
            yCError(PEAK) << "Unable to clear accceptance filters:" << std::strerror(-res);
#endif
            return false;
        }

        activeFilters.clear();
        return true;
    }

    if (activeFilters.erase(_id) == 0)
    {
#if YARP_VERSION_MINOR >= 6
        yCIWarning(PEAK, id()) << "Filter for id" << _id << "missing or already deleted";
#else
        yCWarning(PEAK) << "Filter for id" << _id << "missing or already deleted";
#endif
        return true;
    }

    std::uint64_t acc = computeAcceptanceCodeAndMask();

#if YARP_VERSION_MINOR >= 6
    yCIDebug(PEAK, id(), "New acceptance code+mask: %016lxh", acc);
#else
    yCDebug(PEAK, "New acceptance code+mask: %016lxh", acc);
#endif

    int res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

    if (res < 0)
    {
#if YARP_VERSION_MINOR >= 6
        yCIError(PEAK, id()) << "pcanfd_set_option() failed:" << std::strerror(-res);
#else
        yCError(PEAK) << "pcanfd_set_option() failed:" << std::strerror(-res);
#endif
        activeFilters.insert(_id);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusPeak::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    if (!allowPermissive && wait != blockingMode)
    {
#if YARP_VERSION_MINOR >= 6
        yCIError(PEAK, id(), "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
#else
        yCError(PEAK, "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
#endif
        return false;
    }

    int res;

    {
        std::lock_guard<std::mutex> lockGuard(canBusReady);

        if (blockingMode && rxTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(READ, &bufferReady)) {
#if YARP_VERSION_MINOR >= 6
                yCIError(PEAK, id(), "waitUntilTimeout() failed");
#else
                yCError(PEAK, "waitUntilTimeout() failed");
#endif
                return false;
            }

            if (!bufferReady)
            {
                *read = 0;
                return true;
            }
        }

        // Point at first member of an internally defined array of pcanfd_msg structs.
        struct pcanfd_msg * pfdm = reinterpret_cast<struct pcanfd_msg *>(msgs.getPointer()[0]->getPointer());
        res = pcanfd_recv_msgs_list(fileDescriptor, size, pfdm);
    }

    if (!blockingMode && res == -EWOULDBLOCK)
    {
        *read = 0;
        return true;
    }
    else if (res < 0)
    {
#if YARP_VERSION_MINOR >= 6
        yCIError(PEAK, id(), "Unable to read messages: %s", std::strerror(-res));
#else
        yCError(PEAK, "Unable to read messages: %s", std::strerror(-res));
#endif
        return false;
    }
    else
    {
        *read = res;
        return true;
    }
}

// -----------------------------------------------------------------------------

bool CanBusPeak::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    if (!allowPermissive && wait != blockingMode)
    {
#if YARP_VERSION_MINOR >= 6
        yCIError(PEAK, id(), "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
#else
        yCError(PEAK, "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
#endif
        return false;
    }

    int res;

    {
        std::lock_guard<std::mutex> lockGuard(canBusReady);

        // Point at first member of an internally defined array of pcanfd_msg structs.
        const struct pcanfd_msg * pfdm = reinterpret_cast<const struct pcanfd_msg *>(msgs.getPointer()[0]->getPointer());

        if (blockingMode && txTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(WRITE, &bufferReady)) {
#if YARP_VERSION_MINOR >= 6
                yCIError(PEAK, id(), "waitUntilTimeout() failed");
#else
                yCError(PEAK, "waitUntilTimeout() failed");
#endif
                return false;
            }

            if (!bufferReady)
            {
                *sent = 0;
                return true;
            }
        }

        res = pcanfd_send_msgs_list(fileDescriptor, size, pfdm);
    }

    if (!blockingMode && res == -EWOULDBLOCK)
    {
        *sent = 0;
        return true;
    }
    else if (res < 0)
    {
#if YARP_VERSION_MINOR >= 6
        yCIError(PEAK, id(), "Unable to send messages: %s", std::strerror(-res));
#else
        yCError(PEAK, "Unable to send messages: %s", std::strerror(-res));
#endif
        return false;
    }
    else
    {
        *sent = res;
        return true;
    }
}

// -----------------------------------------------------------------------------
