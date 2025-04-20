// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <cstring> // std::strerror
#include <cerrno>

#include <string>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool CanBusHico::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        yCIError(HICO, id()) << "Could not parse parameters";
        return false;
    }

    yarp::dev::DeviceDriver::setId(m_port);

    if (m_blockingMode)
    {
        yCIInfo(HICO, id()) << "Blocking mode enabled";

        if (m_rxTimeoutMs <= 0)
        {
            yCIWarning(HICO, id()) << "RX timeout value <= 0, CAN read calls will block until the buffer is ready";
        }

        if (m_txTimeoutMs <= 0)
        {
            yCIWarning(HICO, id()) << "TX timeout value <= 0, CAN write calls will block until the buffer is ready";
        }
    }
    else
    {
        yCIInfo(HICO, id()) << "Requested non-blocking mode";
    }

    filterConfig = parseFilterConfiguration(m_filterConfiguration);

    //-- Open the CAN device for reading and writing.
    fileDescriptor = ::open(m_port.c_str(), O_RDWR);

    if (fileDescriptor == -1)
    {
        yCIError(HICO, id()) << "Could not open";
        return false;
    }

    yCIInfo(HICO, id()) << "Successfully opened";

    //-- Set the CAN bitrate.
    if (!canSetBaudRate(m_bitrate))
    {
        yCIError(HICO, id()) << "Could not set bitrate";
        return false;
    }

    yCIInfo(HICO, id()) << "Bitrate set";

    if (!m_blockingMode)
    {
        int fcntlFlags = ::fcntl(fileDescriptor, F_GETFL);

        if (fcntlFlags == -1)
        {
            yCIError(HICO, id()) << "Unable to retrieve FD flags";
            return false;
        }

        fcntlFlags |= O_NONBLOCK;

        if (::fcntl(fileDescriptor, F_SETFL, fcntlFlags) == -1)
        {
            yCIError(HICO, id()) << "Unable to set non-blocking mode:" << std::strerror(errno);
            return false;
        }

        yCIInfo(HICO, id()) << "Non-blocking mode enabled";
    }

    if (filterConfig != FilterManager::DISABLED)
    {
        filterManager = new FilterManager(*this, fileDescriptor, filterConfig == FilterManager::MASK_AND_RANGE);

        if (!m_preserveFilters)
        {
            if (!filterManager->clearFilters())
            {
                yCIError(HICO, id()) << "Unable to clear acceptance filters";
                return false;
            }
            else
            {
                yCIInfo(HICO, id()) << "Acceptance filters cleared";
            }
        }
        else
        {
            yCIWarning(HICO, id()) << "Preserving previous acceptance filters (if any)";
        }

        //-- Load initial node IDs and set acceptance filters.
        if (!m_filteredIds.empty())
        {
            yCIInfo(HICO, id()) << "Parsing filtered ids";

            if (!filterManager->parseIds(m_filteredIds))
            {
                yCIError(HICO, id()) << "Could not set acceptance filters";
                return false;
            }

            if (!filterManager->isValid())
            {
                yCIWarning(HICO, id()) << "Hardware limit was hit and no acceptance filters are enabled";
            }
        }
    }
    else
    {
        yCIInfo(HICO, id()) << "Acceptance filters are disabled";
    }

    //-- Start the CAN device.
    if (::ioctl(fileDescriptor,IOC_START) == -1)
    {
        yCIError(HICO, id()) << "IOC_START failed";
        return false;
    }

    yCIInfo(HICO, id()) << "IOC_START ok";

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusHico::close()
{
    if (fileDescriptor > 0)
    {
        if (filterConfig != FilterManager::DISABLED && filterManager)
        {
            filterManager->clearFilters();
            delete filterManager;
            filterManager = nullptr;
        }

        ::close(fileDescriptor);
        fileDescriptor = 0;
    }

    return true;
}

// -----------------------------------------------------------------------------
