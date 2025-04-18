// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool LacqueyFetch::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(LCQ) << "Failed to parse parameters";
        return false;
    }

    if (m_canId <= 0 || m_canId > 127)
    {
        yCError(LCQ) << "Illegal CAN ID:" << m_canId;
        return false;
    }

    yarp::dev::DeviceDriver::setId("ID" + std::to_string(m_canId));

    if (m_name.empty())
    {
        yCIWarning(LCQ, id()) << "Illegal axis name (empty string)";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::close()
{
    return true;
}

// -----------------------------------------------------------------------------
