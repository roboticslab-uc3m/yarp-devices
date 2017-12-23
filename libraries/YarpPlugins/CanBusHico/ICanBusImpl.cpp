// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <cstring>
#include <vector>
#include <algorithm>

#include <ColorDebug.hpp>

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canSetBaudRate(unsigned int rate)
{
    CD_DEBUG("(%d)\n", rate);

    std::string rateStr;

    if (!interpretBitrate(rate, rateStr))
    {
        CD_ERROR("Unrecognized bitrate value.\n");
        return false;
    }

    CD_INFO("Setting bitrate (%s).\n", rateStr.c_str());

    canBusReady.wait();
    int ret = ::ioctl(fileDescriptor, IOC_SET_BITRATE, &rate);
    canBusReady.post();

    if (ret != 0)
    {
        CD_ERROR("Could not set bitrate: %s.\n", std::strerror(errno));
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canGetBaudRate(unsigned int * rate)
{
    CD_DEBUG("\n");

    canBusReady.wait();
    int ret = ::ioctl(fileDescriptor, IOC_GET_BITRATE, rate);
    canBusReady.post();

    if (ret != 0)
    {
        CD_ERROR("Could not set bitrate: %s.\n", std::strerror(errno));
        return false;
    }

    std::string rateStr;

    if (interpretBitrate(*rate, rateStr))
    {
        CD_DEBUG("Got bitrate: %s (%d).\n", rateStr.c_str(), *rate);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canIdAdd(unsigned int id)
{
    CD_DEBUG("(%d)\n", id);

    if (id > 0x7F)
    {
        CD_ERROR("Invalid ID (%d > 0x7F).\n", id);
        return false;
    }

    canBusReady.wait();

    if (!filteredIds.insert(id).second)
    {
        CD_WARNING("Filter for ID %d is already active.\n", id);
        canBusReady.post();
        return true;
    }

    struct can_filter filter;
    filter.type = FTYPE_AMASK;
    filter.mask = 0x7F;  //-- dsPIC style, mask specifies "do care" bits
    filter.code = id;

    if (::ioctl(fileDescriptor, IOC_SET_FILTER, &filter) != 0)
    {
        CD_ERROR("Could not set filter: %s.\n", std::strerror(errno));
        filteredIds.erase(id);
        canBusReady.post();
        return false;
    }

    canBusReady.post();

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canIdDelete(unsigned int id)
{
    CD_DEBUG("(%d)\n", id);

    if (id > 0x7F)
    {
        CD_ERROR("Invalid ID (%d > 0x7F).\n", id);
        return false;
    }

    canBusReady.wait();

    if (filteredIds.find(id) == filteredIds.end())
    {
        CD_WARNING("Filter for ID %d not found, doing nothing.\n", id);
        canBusReady.post();
        return true;
    }

    if (!clearFilters())
    {
        CD_ERROR("Could not clear list of active filters prior to populating it with previously stored IDs.\n");
        canBusReady.post();
        return false;
    }

    filteredIds.erase(id);

    std::vector<unsigned int> localCopy(filteredIds.begin(), filteredIds.end());

    canBusReady.post();

    for (std::vector<unsigned int>::iterator it = localCopy.begin(); it != localCopy.end(); ++it)
    {
        if (!canIdAdd(*it))
        {
            CD_WARNING("Could not add ID %d back.\n", *it);
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    bool ok = true;

    canBusReady.wait();

    if (!setFdMode(wait))
    {
        CD_WARNING("setFdMode() failed: %s.\n", std::strerror(errno));
    }

    for (unsigned int i = 0; i < size; i++)
    {
        if (wait && !setDelay())
        {
            ok = false;
            break;
        }

        yarp::dev::CanMessage & msg = msgs[i];
        struct can_msg * _msg = reinterpret_cast<struct can_msg *>(msg.getPointer());

        //-- read() returns the number read, -1 for errors or 0 for EOF.
        if (::read(fileDescriptor, _msg, sizeof(struct can_msg)) < 0)
        {
            CD_WARNING("read() error: %s.\n", std::strerror(errno));
        }
        else
        {
            (*read)++;
        }
    }

    if (*read < size)
    {
        CD_ERROR("read (%d) < size (%d)\n", *read, size);
        ok = false;
    }

    canBusReady.post();

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    bool ok = true;

    canBusReady.wait();

    if (!setFdMode(wait))
    {
        CD_WARNING("setFdMode() failed: %s.\n", std::strerror(errno));
    }

    for (unsigned int i = 0; i < size; i++)
    {
        // 'wait' param not handled

        const yarp::dev::CanMessage & msg = const_cast<yarp::dev::CanBuffer &>(msgs)[i];
        const struct can_msg * _msg = reinterpret_cast<const struct can_msg *>(msg.getPointer());

        if (::write(fileDescriptor, _msg, sizeof(struct can_msg)) == -1)
        {
            CD_WARNING("%s.\n", std::strerror(errno));
        }
        else
        {
            (*sent)++;
        }
    }

    if (*sent < size)
    {
        CD_ERROR("sent (%d) < size (%d)\n", *sent, size);
        ok = false;
    }

    canBusReady.post();

    return ok;
}

// -----------------------------------------------------------------------------
