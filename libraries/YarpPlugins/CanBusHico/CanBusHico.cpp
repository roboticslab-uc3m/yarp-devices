// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <sys/select.h>
#include <sys/time.h>

#include <cstring>
#include <cerrno>
#include <cassert>

#include <map>
#include <stdexcept>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    void setTimeval(int timeMs, struct timeval * tv)
    {
        tv->tv_sec = timeMs / 1000;
        tv->tv_usec = (timeMs % 1000) * 1000;
    }

    const std::map<unsigned int, unsigned int> idToBitrateMap {
        {BITRATE_10k, 10000},
        {BITRATE_20k, 20000},
        {BITRATE_50k, 50000},
        {BITRATE_100k, 100000},
        {BITRATE_125k, 125000},
        {BITRATE_250k, 250000},
        {BITRATE_500k, 500000},
        {BITRATE_800k, 800000},
        {BITRATE_1000k, 1000000}
    };
}

// -----------------------------------------------------------------------------

bool CanBusHico::waitUntilTimeout(io_operation op, bool * bufferReady)
{
    fd_set fds;

    FD_ZERO(&fds);
    FD_SET(fileDescriptor, &fds);

    struct timeval tv;

    //-- select() returns the number of ready descriptors, 0 for timeout, -1 for errors.
    int ret;

    switch (op)
    {
    case READ:
        setTimeval(rxTimeoutMs, &tv);
        ret = ::select(fileDescriptor + 1, &fds, 0, 0, &tv);
        break;
    case WRITE:
        setTimeval(txTimeoutMs, &tv);
        ret = ::select(fileDescriptor + 1, 0, &fds, 0, &tv);
        break;
    default:
        yCIError(HICO, id(), "Unhandled IO operation on select()");
        return false;
    }

    if (ret < 0)
    {
        yCIError(HICO, id(), "select() error: %s", std::strerror(errno));
        return false;
    }
    else if (ret == 0)
    {
        *bufferReady = false;
    }
    else
    {
        assert(FD_ISSET(fileDescriptor, &fds));
        *bufferReady = true;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::bitrateToId(unsigned int bitrate, unsigned int * id)
{
    for (const auto & [_id, _bitrate] : idToBitrateMap)
    {
        if (_bitrate == bitrate)
        {
            *id = _id;
            return true;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------

bool CanBusHico::idToBitrate(unsigned int id, unsigned int * bitrate)
{
    try
    {
        *bitrate = idToBitrateMap.at(id);
        return true;
    }
    catch (const std::out_of_range & exception)
    {
        return false;
    }
}

// -----------------------------------------------------------------------------

CanBusHico::FilterManager::filter_config CanBusHico::parseFilterConfiguration(const std::string & str)
{
    if (str == "disabled")
    {
        return FilterManager::DISABLED;
    }
    else if (str == "noRange")
    {
        return FilterManager::NO_RANGE;
    }
    else if (str == "maskAndRange")
    {
        return FilterManager::MASK_AND_RANGE;
    }
    else
    {
        yCIWarning(HICO, id()) << "Unrecognized filter configuration, setting DISABLED:" << str;
        return FilterManager::DISABLED;
    }
}

// -----------------------------------------------------------------------------
