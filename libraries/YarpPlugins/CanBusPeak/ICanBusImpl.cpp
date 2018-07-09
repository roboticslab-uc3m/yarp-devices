// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusPeak.hpp"

#include <cstring> // std::memset, std::strerror

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
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    return false;
}

// -----------------------------------------------------------------------------
