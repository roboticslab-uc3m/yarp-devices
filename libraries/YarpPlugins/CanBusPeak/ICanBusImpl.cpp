// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusPeak.hpp"

#include <cstring> // std::memset, std::memcpy, std::strerror
#include <cerrno> // error codes

#include <libpcanfd.h>

#include <yarp/os/LogStream.h>

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canSetBaudRate(unsigned int rate)
{
    struct pcanfd_init pfdi;
    std::memset(&pfdi, '\0', sizeof(pfdi));
    pfdi.nominal.bitrate = rate;

    canBusReady.lock();
    int res = pcanfd_set_init(fileDescriptor, &pfdi);
    canBusReady.unlock();

    if (res < 0)
    {
        yError() << "Unable to set bitrate:" << std::strerror(-res);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canGetBaudRate(unsigned int * rate)
{
    struct pcanfd_init pfdi;

    canBusReady.lock();
    int res = pcanfd_get_init(fileDescriptor, &pfdi);
    canBusReady.unlock();

    if (res < 0)
    {
        yError() << "Unable to retrieve bitrate:" << std::strerror(-res);
        return false;
    }

    *rate = pfdi.nominal.bitrate;

    if (pfdi.nominal.bitrate != pfdi.nominal.bitrate_real)
    {
        yWarning() << "User-defined nominal bitrate" << pfdi.nominal.bitrate << "differs from real nominal bitrate" << pfdi.nominal.bitrate_real;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canIdAdd(unsigned int id)
{
    std::lock_guard<std::mutex> lockGuard(canBusReady);

    if (activeFilters.find(id) != activeFilters.end())
    {
        yWarning() << "Filter for id" << id << "already set";
        return true;
    }

    activeFilters.insert(id);

    std::uint64_t acc = computeAcceptanceCodeAndMask();

    yDebug("New acceptance code+mask: %016lxh", acc);

    int res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

    if (res < 0)
    {
        yError() << "pcanfd_set_option() failed:" << std::strerror(-res);
        activeFilters.erase(id);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canIdDelete(unsigned int id)
{
    std::lock_guard<std::mutex> lockGuard(canBusReady);

    if (id == 0)
    {
        yInfo() << "Clearing filters previously set";

        int res = pcanfd_del_filters(fileDescriptor);

        if (res < 0)
        {
            yError() << "Unable to clear accceptance filters:" << std::strerror(-res);
            return false;
        }

        activeFilters.clear();
        return true;
    }

    if (activeFilters.erase(id) == 0)
    {
        yWarning() << "Filter for id" << id << "missing or already deleted";
        return true;
    }

    std::uint64_t acc = computeAcceptanceCodeAndMask();

    yDebug("New acceptance code+mask: %016lxh", acc);

    int res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

    if (res < 0)
    {
        yError() << "pcanfd_set_option() failed:" << std::strerror(-res);
        activeFilters.insert(id);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    if (!allowPermissive && wait != blockingMode)
    {
        yError("Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
        return false;
    }

    int res;

    {
        std::lock_guard<std::mutex> lockGuard(canBusReady);

        if (blockingMode && rxTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(READ, &bufferReady)) {
                yError("waitUntilTimeout() failed");
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
        yError("Unable to read messages: %s", std::strerror(-res));
        return false;
    }
    else
    {
        *read = res;
        return true;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    if (!allowPermissive && wait != blockingMode)
    {
        yError("Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, blockingMode);
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
                yError("waitUntilTimeout() failed");
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
        yError("Unable to send messages: %s", std::strerror(-res));
        return false;
    }
    else
    {
        *sent = res;
        return true;
    }
}

// -----------------------------------------------------------------------------
