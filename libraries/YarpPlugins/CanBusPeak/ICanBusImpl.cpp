// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusPeak.hpp"

#include <cstring> // std::memset, std::memcpy, std::strerror
#include <cerrno> // error codes

#include <libpcanfd.h>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canSetBaudRate(unsigned int rate)
{
    CD_DEBUG("(%d)\n", rate);

    struct pcanfd_init pfdi;
    std::memset(&pfdi, '\0', sizeof(pfdi));
    pfdi.nominal.bitrate = rate;

    canBusReady.lock();
    int res = pcanfd_set_init(fileDescriptor, &pfdi);
    canBusReady.unlock();

    if (res < 0)
    {
        CD_ERROR("Unable to set bitrate (%s).\n", std::strerror(-res));
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
        CD_ERROR("Unable to retrieve bitrate (%s).\n", std::strerror(-res));
        return false;
    }

    *rate = pfdi.nominal.bitrate;

    if (pfdi.nominal.bitrate != pfdi.nominal.bitrate_real)
    {
        CD_WARNING("User-defined nominal bitrate (%d) differs from real nominal bitrate (%d).\n",
                   pfdi.nominal.bitrate, pfdi.nominal.bitrate_real);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canIdAdd(unsigned int id)
{
    CD_DEBUG("(%d)\n", id);

    std::lock_guard<std::mutex> lockGuard(canBusReady);

    if (activeFilters.find(id) != activeFilters.end())
    {
        CD_WARNING("Filter for id %d already set.\n", id);
        return true;
    }

    activeFilters.insert(id);

    std::uint64_t acc = computeAcceptanceCodeAndMask();

    CD_DEBUG("New acceptance code+mask: %016lxh.\n", acc);

    int res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

    if (res < 0)
    {
        CD_ERROR("pcanfd_set_option() failed (%s).\n", std::strerror(-res));
        activeFilters.erase(id);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canIdDelete(unsigned int id)
{
    CD_DEBUG("(%d)\n", id);

    std::lock_guard<std::mutex> lockGuard(canBusReady);

    if (id == 0)
    {
        CD_INFO("Clearing filters previously set.\n");

        int res = pcanfd_del_filters(fileDescriptor);

        if (res < 0)
        {
            CD_ERROR("Unable to clear accceptance filters (%s).\n", std::strerror(-res));
            return false;
        }

        activeFilters.clear();
        return true;
    }

    if (activeFilters.erase(id) == 0)
    {
        CD_WARNING("Filter for id %d missing or already deleted.\n", id);
        return true;
    }

    std::uint64_t acc = computeAcceptanceCodeAndMask();

    CD_DEBUG("New acceptance code+mask: %016lxh.\n", acc);

    int res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

    if (res < 0)
    {
        CD_ERROR("pcanfd_set_option() failed (%s).\n", std::strerror(-res));
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
        CD_ERROR("Blocking mode configuration mismatch: requested=%d, enabled=%d.\n", wait, blockingMode);
        return false;
    }

    int res;

    {
        std::lock_guard<std::mutex> lockGuard(canBusReady);

        if (blockingMode && rxTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(READ, &bufferReady)) {
                CD_ERROR("waitUntilTimeout() failed.\n");
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
        CD_ERROR("Unable to read messages: %s.\n", std::strerror(-res));
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
        CD_ERROR("Blocking mode configuration mismatch: requested=%d, enabled=%d.\n", wait, blockingMode);
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
                CD_ERROR("waitUntilTimeout() failed.\n");
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
        CD_ERROR("Unable to send messages: %s.\n", std::strerror(-res));
        return false;
    }
    else
    {
        *sent = res;
        return true;
    }
}

// -----------------------------------------------------------------------------
