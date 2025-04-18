// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusPeak.hpp"

#include <cstring> // std::memset, std::memcpy, std::strerror
#include <cerrno> // error codes

#include <libpcanfd.h>

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
        yCIError(PEAK, id()) << "Unable to set bitrate:" << std::strerror(-res);
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
        yCIError(PEAK, id()) << "Unable to retrieve bitrate:" << std::strerror(-res);
        return false;
    }

    *rate = pfdi.nominal.bitrate;

    if (pfdi.nominal.bitrate != pfdi.nominal.bitrate_real)
    {
        yCIWarning(PEAK, id()) << "User-defined nominal bitrate" << pfdi.nominal.bitrate << "differs from real nominal bitrate" << pfdi.nominal.bitrate_real;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusPeak::canIdAdd(unsigned int _id)
{
    std::lock_guard lockGuard(canBusReady);

    if (activeFilters.find(_id) != activeFilters.end())
    {
        yCIWarning(PEAK, id()) << "Filter for id" << _id << "already set";
        return true;
    }

    activeFilters.insert(_id);

    std::uint64_t acc = computeAcceptanceCodeAndMask();

    yCIDebug(PEAK, id(), "New acceptance code+mask: %016lxh", acc);

    int res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

    if (res < 0)
    {
        yCIError(PEAK, id()) << "pcanfd_set_option() failed:" << std::strerror(-res);
        activeFilters.erase(_id);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusPeak::canIdDelete(unsigned int _id)
{
    std::lock_guard lockGuard(canBusReady);

    if (_id == 0)
    {
        yCIInfo(PEAK, id()) << "Clearing filters previously set";

        int res = pcanfd_del_filters(fileDescriptor);

        if (res < 0)
        {
            yCIError(PEAK, id()) << "Unable to clear accceptance filters:" << std::strerror(-res);
            return false;
        }

        activeFilters.clear();
        return true;
    }

    if (activeFilters.erase(_id) == 0)
    {
        yCIWarning(PEAK, id()) << "Filter for id" << _id << "missing or already deleted";
        return true;
    }

    std::uint64_t acc = computeAcceptanceCodeAndMask();

    yCIDebug(PEAK, id(), "New acceptance code+mask: %016lxh", acc);

    int res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

    if (res < 0)
    {
        yCIError(PEAK, id()) << "pcanfd_set_option() failed:" << std::strerror(-res);
        activeFilters.insert(_id);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusPeak::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    if (!m_allowPermissive && wait != m_blockingMode)
    {
        yCIError(PEAK, id(), "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, m_blockingMode);
        return false;
    }

    int res;

    {
        std::lock_guard lockGuard(canBusReady);

        if (m_blockingMode && m_rxTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(READ, &bufferReady)) {
                yCIError(PEAK, id(), "waitUntilTimeout() failed");
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

    if (!m_blockingMode && res == -EWOULDBLOCK)
    {
        *read = 0;
        return true;
    }
    else if (res < 0)
    {
        yCIError(PEAK, id(), "Unable to read messages: %s", std::strerror(-res));
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
    if (!m_allowPermissive && wait != m_blockingMode)
    {
        yCIError(PEAK, id(), "Blocking mode configuration mismatch: requested=%d, enabled=%d", wait, m_blockingMode);
        return false;
    }

    int res;

    {
        std::lock_guard lockGuard(canBusReady);

        // Point at first member of an internally defined array of pcanfd_msg structs.
        const struct pcanfd_msg * pfdm = reinterpret_cast<const struct pcanfd_msg *>(msgs.getPointer()[0]->getPointer());

        if (m_blockingMode && m_txTimeoutMs > 0)
        {
            bool bufferReady;

            if (!waitUntilTimeout(WRITE, &bufferReady)) {
                yCIError(PEAK, id(), "waitUntilTimeout() failed");
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

    if (!m_blockingMode && res == -EWOULDBLOCK)
    {
        *sent = 0;
        return true;
    }
    else if (res < 0)
    {
        yCIError(PEAK, id(), "Unable to send messages: %s", std::strerror(-res));
        return false;
    }
    else
    {
        *sent = res;
        return true;
    }
}

// -----------------------------------------------------------------------------
