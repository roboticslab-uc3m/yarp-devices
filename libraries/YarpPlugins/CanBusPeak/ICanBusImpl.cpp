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

    struct pcanfd_msg_filter pf;
    pf.id_from = pf.id_to = id;
    pf.msg_flags = MSGTYPE_STANDARD; // default, see pcan.h

    int res = pcanfd_add_filter(fileDescriptor, &pf);

    if (res < 0)
    {
        CD_ERROR("Unable to set filter: %d (%s).\n", id, std::strerror(-res));
        canBusReady.post();
        return false;
    }

    activeFilters.insert(id);

    canBusReady.post();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canIdDelete(unsigned int id)
{
    CD_DEBUG("(%d)\n", id);

    canBusReady.wait();

    std::set<unsigned int>::const_iterator filterId = activeFilters.find(id);

    if (filterId == activeFilters.end())
    {
        CD_WARNING("Filter for id %d missing or already deleted.\n", id);
        canBusReady.post();
        return true;
    }

    int res = pcanfd_del_filters(fileDescriptor);

    if (res < 0)
    {
        CD_ERROR("Unable to delete all filters (%s).\n", std::strerror(-res));
        canBusReady.post();
        return false;
    }

    activeFilters.erase(filterId);

    if (activeFilters.empty())
    {
        canBusReady.post();
        return true;
    }

    struct pcanfd_msg_filter * pfl = new pcanfd_msg_filter[activeFilters.size()];
    std::set<unsigned int>::const_iterator it;
    int i = 0;

    for (it = activeFilters.begin(); it != activeFilters.end(); ++it)
    {
        pfl[i].id_from = pfl[i].id_to = *it;
        pfl[i].msg_flags = MSGTYPE_STANDARD; // default, see pcan.h
        i++;
    }

    res = pcanfd_add_filters_list(fileDescriptor, activeFilters.size(), pfl);

    if (res < 0)
    {
        CD_ERROR("Unable to add active filters back (%s).\n", std::strerror(-res));
        delete[] pfl;
        activeFilters.clear();
        canBusReady.post();
        return false;
    }

    delete[] pfl;

    canBusReady.post();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    if (wait != !nonBlockingMode)
    {
        CD_ERROR("Blocking mode configuration mismatch: requested=%d, enabled=%d.\n", wait, !nonBlockingMode);
        return false;
    }

    struct pcanfd_msg * pfdm = new struct pcanfd_msg[size];

    canBusReady.wait();

    if (wait && rxTimeoutMs > 0)
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

    if (nonBlockingMode && res == -EWOULDBLOCK)
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
    if (wait != !nonBlockingMode)
    {
        CD_ERROR("Blocking mode configuration mismatch: requested=%d, enabled=%d.\n", wait, !nonBlockingMode);
        return false;
    }

    struct pcanfd_msg * pfdm = new struct pcanfd_msg[size];

    for (unsigned int i = 0; i < size; i++)
    {
        const yarp::dev::CanMessage & msg = const_cast<yarp::dev::CanBuffer &>(msgs)[i];
        std::memcpy(pfdm[i].data, msg.getData(), msg.getLen());
        pfdm[i].data_len = msg.getLen();
        pfdm[i].id = msg.getId();
        pfdm[i].type = PCANFD_TYPE_CAN20_MSG;
    }

    canBusReady.wait();

    if (wait && txTimeoutMs > 0)
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

    if (nonBlockingMode && res == -EWOULDBLOCK)
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
