// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusPeak.hpp"

#include <cstring> // std::strerror

#include <string>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool CanBusPeak::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        yCIError(PEAK, id()) << "Unable to parse parameters";
        return false;
    }

    yarp::dev::DeviceDriver::setId(m_port);

    int flags = OFD_BITRATE | PCANFD_INIT_STD_MSG_ONLY;

    if (m_blockingMode)
    {
        yCIInfo(PEAK, id()) << "Blocking mode enabled";

        if (m_rxTimeoutMs <= 0)
        {
            yCIWarning(PEAK, id()) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready";
        }

        if (m_txTimeoutMs <= 0)
        {
            yCIWarning(PEAK, id()) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready";
        }
    }
    else
    {
        yCIInfo(PEAK, id()) << "Requested non-blocking mode";
        flags |= OFD_NONBLOCKING;
    }

    int res = pcanfd_open(m_port.c_str(), flags, m_bitrate);

    if (res < 0)
    {
        yCIError(PEAK, id()) << "Unable to open:" << std::strerror(-res);
        return false;
    }
    else
    {
        yCIInfo(PEAK, id()) << "Successfully opened";
        fileDescriptor = res;
    }

    if (!m_preserveFilters)
    {
        res = pcanfd_del_filters(fileDescriptor);

        if (res < 0)
        {
            yCIError(PEAK, id()) << "Unable to clear acceptance filters:" << std::strerror(-res);
            return false;
        }
        else
        {
            yCIInfo(PEAK, id()) << "Acceptance filters cleared";
        }
    }
    else
    {
        yCIWarning(PEAK, id()) << "Preserving previous acceptance filters (if any)";
    }

    //-- Load initial node IDs and set acceptance filters.
    if (!m_filteredIds.empty())
    {
        yCIInfo(PEAK, id()) << "Parsing filtered ids";

        for (const auto id : m_filteredIds)
        {
            activeFilters.insert(id);
        }

        std::uint64_t acc = computeAcceptanceCodeAndMask();

        yCIDebug(PEAK, id(), "New acceptance code+mask: %016lxh", acc);

        res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

        if (res < 0)
        {
            yCIError(PEAK, id()) << "Unable to set acceptance filters:" << std::strerror(-res);
            activeFilters.clear();
            return false;
        }

        yCIInfo(PEAK, id()) << "Initial IDs added to set of acceptance filters";
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusPeak::close()
{
    if (fileDescriptor > 0)
    {
        if (!activeFilters.empty())
        {
            int res = pcanfd_del_filters(fileDescriptor);

            if (res < 0)
            {
                yCIWarning(PEAK, id()) << "Unable to clear acceptance filters:" << std::strerror(-res);
            }
            else
            {
                activeFilters.clear();
            }
        }

        pcanfd_close(fileDescriptor);
        fileDescriptor = 0;
    }

    return true;
}

// -----------------------------------------------------------------------------
