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

    const auto & commonGroup = robotConfig->findGroup("common-ipos");
    yarp::os::Property iposGroup;

    if (!commonGroup.isNull())
    {
        yCDebugOnce(IPOS) << commonGroup.toString();
        iposGroup.fromString(commonGroup.toString());
    }

    iposGroup.fromString(config.toString(), false); // override common options

    auto useEmbeddedPid = iposGroup.check("useEmbeddedPid", yarp::os::Value(true), "use embedded PID").asBool();

    if (useEmbeddedPid)
    {
        yCInfo(IPOS) << "Using embedded PID implementation";
        impl = new TechnosoftIposEmbedded;
    }
    else
    {
#ifdef HAVE_EXTERNAL_PID_IMPL
        yCInfo(IPOS) << "Using external PID implementation";
        impl = new TechnosoftIposExternal;
#else
        yCError(IPOS) << "External PID implementation not available";
        return false;
#endif
    }

    return impl->open(config);
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
