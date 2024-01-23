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

constexpr auto DEFAULT_USE_EMBEDDED = true;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::open(yarp::os::Searchable & config)
{
    yarp::os::Property options;
    options.fromString(config.findGroup("common").toString());
    options.fromString(config.toString(), false); // override common options

    if (options.check("useEmbeddedPid", yarp::os::Value(DEFAULT_USE_EMBEDDED), "use embedded PID").asBool())
    {
        yCIInfo(IPOS, id()) << "Using embedded PID implementation";
        impl = new TechnosoftIposEmbedded;
    }
    else
    {
#ifdef HAVE_EXTERNAL_PID_IMPL
        yCIInfo(IPOS, id()) << "Using external PID implementation";
        impl = new TechnosoftIposExternal;
#else
        yCError(IPOS) << "External PID implementation not available";
        return false;
#endif
    }

    impl->setId(id());
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
