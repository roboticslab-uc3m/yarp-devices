// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <sys/select.h>
#include <sys/time.h>

#include <cstring>
#include <cerrno>
#include <cassert>

#include <stdexcept>

#include <yarp/conf/version.h>
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
}

// -----------------------------------------------------------------------------

std::map<unsigned int, unsigned int> CanBusHico::idToBitrateMap;

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
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id(), "Unhandled IO operation on select()");
#else
        yCError(HICO, "Unhandled IO operation on select()");
#endif
        return false;
    }

    if (ret < 0)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(HICO, id(), "select() error: %s", std::strerror(errno));
#else
        yCError(HICO, "select() error: %s", std::strerror(errno));
#endif
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

void CanBusHico::initBitrateMap()
{
    idToBitrateMap[BITRATE_10k] = 10000;
    idToBitrateMap[BITRATE_20k] = 20000;
    idToBitrateMap[BITRATE_50k] = 50000;
    idToBitrateMap[BITRATE_100k] = 100000;
    idToBitrateMap[BITRATE_125k] = 125000;
    idToBitrateMap[BITRATE_250k] = 250000;
    idToBitrateMap[BITRATE_500k] = 500000;
    idToBitrateMap[BITRATE_800k] = 800000;
    idToBitrateMap[BITRATE_1000k] = 1000000;
}

// -----------------------------------------------------------------------------

bool CanBusHico::bitrateToId(unsigned int bitrate, unsigned int * id)
{
    std::map<unsigned int, unsigned int>::const_iterator it;

    for (it = idToBitrateMap.begin(); it != idToBitrateMap.end(); ++it)
    {
        if (it->second == bitrate)
        {
            *id = it->first;
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
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIWarning(HICO, id()) << "Unrecognized filter configuration, setting DISABLED:" << str;
#else
        yCWarning(HICO) << "Unrecognized filter configuration, setting DISABLED:" << str;
#endif
        return FilterManager::DISABLED;
    }
}

// -----------------------------------------------------------------------------
