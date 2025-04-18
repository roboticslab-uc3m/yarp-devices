// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <cmath>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CuiAbsolute::open(yarp::os::Searchable & config)
{
    if (!config.check("robotConfig") || !config.find("robotConfig").isBlob())
    {
        yCError(CUI) << "Missing \"robotConfig\" property or not a blob";
        return false;
    }

    const auto * robotConfig = *reinterpret_cast<const yarp::os::Property * const *>(config.find("robotConfig").asBlob());

    yarp::os::Property fullConfig;

    fullConfig.fromString(robotConfig->findGroup("common-cui").toString());
    fullConfig.fromString(config.toString(), false); // override common options

    if (!parseParams(fullConfig))
    {
        yCIError(CUI, id()) << "Could not parse parameters";
        return false;
    }

    if (m_canId <= 0 || m_canId > 127)
    {
        yCError(CUI) << "Illegal CAN ID:" << m_canId;
        return false;
    }

    yarp::dev::DeviceDriver::setId("ID" + std::to_string(m_canId));

    if (m_timeout <= 0.0)
    {
        yCIError(CUI, id()) << "Illegal CUI timeout value:" << m_timeout;
        return false;
    }

    if (m_mode == "push")
    {
        pushStateObserver = new StateObserver(m_timeout);
        cuiMode = CuiMode::PUSH;
    }
    else if (m_mode == "pull")
    {
        pollStateObserver = new TypedStateObserver<encoder_t>(m_timeout);
        cuiMode = CuiMode::PULL;
    }
    else
    {
        yCIError(CUI, id()) << "Unrecognized CUI mode:" << m_mode;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::close()
{
    if (pushStateObserver)
    {
        delete pushStateObserver;
        pushStateObserver = nullptr;
    }

    if (pollStateObserver)
    {
        delete pollStateObserver;
        pollStateObserver = nullptr;
    }

    return true;
}

// -----------------------------------------------------------------------------
