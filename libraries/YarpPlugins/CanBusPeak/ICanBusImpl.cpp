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

    canBusReady.wait();
    int res = pcanfd_set_init(fileDescriptor, &pfdi);
    canBusReady.post();

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

    canBusReady.wait();
    int res = pcanfd_get_init(fileDescriptor, &pfdi);
    canBusReady.post();

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

    canBusReady.wait();

    if (activeFilters.find(id) != activeFilters.end())
    {
        CD_WARNING("Filter for id %d already set.\n", id);
        canBusReady.post();
        return true;
    }

    activeFilters.insert(id);

    uint64_t acc = computeAcceptanceCodeAndMask();

    CD_DEBUG("New acceptance code+mask: %016lxh.\n", acc);

    int res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

    if (res < 0)
    {
        CD_ERROR("pcanfd_set_option() failed (%s).\n", std::strerror(-res));
        activeFilters.erase(id);
        canBusReady.post();
        return false;
    }

    canBusReady.post();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canIdDelete(unsigned int id)
{
    CD_DEBUG("(%d)\n", id);

    canBusReady.wait();

    if (id == 0)
    {
        CD_INFO("Clearing filters previously set.\n");

        int res = pcanfd_del_filters(fileDescriptor);

        if (res < 0)
        {
            CD_ERROR("Unable to clear accceptance filters (%s).\n", std::strerror(-res));
            canBusReady.post();
            return false;
        }

        activeFilters.clear();
        canBusReady.post();
        return true;
    }

    if (activeFilters.erase(id) == 0)
    {
        CD_WARNING("Filter for id %d missing or already deleted.\n", id);
        canBusReady.post();
        return true;
    }

    uint64_t acc = computeAcceptanceCodeAndMask();

    CD_DEBUG("New acceptance code+mask: %016lxh.\n", acc);

    int res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

    if (res < 0)
    {
        CD_ERROR("pcanfd_set_option() failed (%s).\n", std::strerror(-res));
        activeFilters.insert(id);
        canBusReady.post();
        return false;
    }

    canBusReady.post();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    if (wait != blockingMode)
    {
        CD_ERROR("Blocking mode configuration mismatch: requested=%d, enabled=%d.\n", wait, blockingMode);
        return false;
    }

    struct pcanfd_msg * pfdm = new struct pcanfd_msg[size];

    canBusReady.wait();

    if (blockingMode && rxTimeoutMs > 0)
    {
        bool bufferReady;

        if (!waitUntilTimeout(READ, &bufferReady)) {
            canBusReady.post();
            CD_ERROR("waitUntilTimeout() failed.\n");
            delete[] pfdm;
            return false;
        }

        if (!bufferReady)
        {
            canBusReady.post();
            *read = 0;
            delete[] pfdm;
            return true;
        }
    }

    int res = pcanfd_recv_msgs_list(fileDescriptor, size, pfdm);

    canBusReady.post();

    if (!blockingMode && res == -EWOULDBLOCK)
    {
        *read = 0;
        delete[] pfdm;
        return true;
    }
    else if (res < 0)
    {
        CD_ERROR("Unable to read messages: %s.\n", std::strerror(-res));
        delete[] pfdm;
        return false;
    }

    *read = res;

    for (unsigned int i = 0; i < res; i++)
    {
        yarp::dev::CanMessage & msg = msgs[i];
        std::memcpy(msg.getData(), pfdm[i].data, pfdm[i].data_len);
        msg.setLen(pfdm[i].data_len);
        msg.setId(pfdm[i].id);
    }

    delete[] pfdm;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    if (wait != blockingMode)
    {
        CD_ERROR("Blocking mode configuration mismatch: requested=%d, enabled=%d.\n", wait, blockingMode);
        return false;
    }

    struct pcanfd_msg * pfdm = new struct pcanfd_msg[size];

    for (unsigned int i = 0; i < size; i++)
    {
        const yarp::dev::CanMessage & msg = msgs[i];
        std::memcpy(pfdm[i].data, msg.getData(), msg.getLen());
        pfdm[i].data_len = msg.getLen();
        pfdm[i].id = msg.getId();
        pfdm[i].type = PCANFD_TYPE_CAN20_MSG;
        pfdm[i].flags = PCANFD_MSG_STD;
    }

    canBusReady.wait();

    if (blockingMode && txTimeoutMs > 0)
    {
        bool bufferReady;

        if (!waitUntilTimeout(WRITE, &bufferReady)) {
            canBusReady.post();
            CD_ERROR("waitUntilTimeout() failed.\n");
            delete[] pfdm;
            return false;
        }

        if (!bufferReady)
        {
            canBusReady.post();
            *sent = 0;
            delete[] pfdm;
            return true;
        }
    }

    int res = pcanfd_send_msgs_list(fileDescriptor, size, pfdm);

    canBusReady.post();

    delete[] pfdm;

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

    *sent = res;

    return true;
}

// -----------------------------------------------------------------------------
