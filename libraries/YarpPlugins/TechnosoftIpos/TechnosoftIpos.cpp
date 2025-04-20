// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#ifdef HAVE_EXTERNAL_PID_IMPL
# include "external-pid/TechnosoftIposExternal.hpp"
#endif

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::open(yarp::os::Searchable & config)
{
    if (!config.check("robotConfig") || !config.find("robotConfig").isBlob())
    {
        yCError(IPOS) << "Missing \"robotConfig\" property or not a blob";
        return false;
    }

    const auto * robotConfig = *reinterpret_cast<const yarp::os::Property * const *>(config.find("robotConfig").asBlob());

    yarp::os::Property fullConfig;

    fullConfig.fromString(robotConfig->findGroup("common-ipos").toString());
    fullConfig.fromString(config.toString(), false); // override common options

    const auto & driverGroup = robotConfig->findGroup(fullConfig.find("driver").asString());
    const auto & motorGroup = robotConfig->findGroup(fullConfig.find("motor").asString());
    const auto & gearboxGroup = robotConfig->findGroup(fullConfig.find("gearbox").asString());
    const auto & encoderGroup = robotConfig->findGroup(fullConfig.find("encoder").asString());

    fullConfig.addGroup("driver").fromString(driverGroup.toString());
    fullConfig.addGroup("motor").fromString(motorGroup.toString());
    fullConfig.addGroup("gearbox").fromString(gearboxGroup.toString());
    fullConfig.addGroup("encoder").fromString(encoderGroup.toString());

    yCDebug(IPOS) << fullConfig.toString();

    if (!parseParams(fullConfig))
    {
        yCError(IPOS) << "Could not parse parameters";
        return false;
    }

    if (m_canId <= 0 || m_canId > 127)
    {
        yCError(IPOS) << "Illegal CAN ID:" << m_canId;
        return false;
    }

    const auto id = "ID" + std::to_string(m_canId);

    if (m_useEmbeddedPid)
    {
        yCIInfo(IPOS, id) << "Using embedded PID implementation";
        impl = new TechnosoftIposEmbedded(*this);
    }
    else
    {
#ifdef HAVE_EXTERNAL_PID_IMPL
        yCIInfo(IPOS, id) << "Using external PID implementation";
        impl = new TechnosoftIposExternal(*this);
#else
        yCIError(IPOS, id) << "External PID implementation not available";
        return false;
#endif
    }

    impl->setId(id);
    return impl->open(fullConfig);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::close()
{
    if (impl)
    {
        bool ret = impl->close();
        delete impl;
        impl = nullptr;
        return ret;
    }

    return true;
}

// -----------------------------------------------------------------------------
