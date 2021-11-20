// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <cmath>

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_TIMEOUT = 0.25; // [s]
constexpr auto DEFAULT_MAX_RETRIES = 10;

// -----------------------------------------------------------------------------

bool CuiAbsolute::open(yarp::os::Searchable & config)
{
    if (!config.check("robotConfig") || !config.find("robotConfig").isBlob())
    {
        yCError(CUI) << "Missing \"robotConfig\" property or not a blob";
        return false;
    }

    const auto * robotConfig = *reinterpret_cast<const yarp::os::Property * const *>(config.find("robotConfig").asBlob());

    yarp::os::Bottle & commonGroup = robotConfig->findGroup("common-cui");
    yarp::os::Property cuiGroup;

    if (!commonGroup.isNull())
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCDebugOnce(IPOS) << commonGroup.toString();
#endif
        cuiGroup.fromString(commonGroup.toString());
    }

    cuiGroup.fromString(config.toString(), false); // override common options

#if !defined(YARP_VERSION_COMPARE) // < 3.6.0
    yCDebug(CUI) << "Config:" << cuiGroup.toString();
#endif

    canId = config.check("canId", yarp::os::Value(0), "CAN bus ID").asInt8(); // id-specific
    reverse = cuiGroup.check("reverse", yarp::os::Value(false), "reverse").asBool();
    timeout = cuiGroup.check("timeout", yarp::os::Value(DEFAULT_TIMEOUT), "timeout (seconds)").asFloat64();
    maxRetries = cuiGroup.check("maxRetries", yarp::os::Value(DEFAULT_MAX_RETRIES), "max retries on timeout").asFloat64();

    if (canId <= 0)
    {
        yCError(CUI) << "Illegal CAN ID:" << canId;
        return false;
    }

#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
    yarp::dev::DeviceDriver::setId("ID" + std::to_string(canId));
#endif

    if (timeout <= 0.0)
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(CUI, id()) << "Illegal CUI timeout value:" << timeout;
#else
        yCError(CUI) << "Illegal CUI timeout value:" << timeout;
#endif
        return false;
    }

    if (!cuiGroup.check("mode", "publish mode [push|pull]"))
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(CUI, id()) << "Missing \"mode\" property";
#else
        yCError(CUI) << "Missing \"mode\" property";
#endif
        return false;
    }

    std::string mode = cuiGroup.find("mode").asString();

    if (mode == "push")
    {
        pushStateObserver = new StateObserver(timeout);
        cuiMode = CuiMode::PUSH;
        pushDelay = cuiGroup.check("pushDelay", yarp::os::Value(0), "Cui push mode delay [0-255]").asInt8();
    }
    else if (mode == "pull")
    {
        pollStateObserver = new TypedStateObserver<encoder_t>(timeout);
        cuiMode = CuiMode::PULL;
    }
    else
    {
#if defined(YARP_VERSION_COMPARE) // >= 3.6.0
        yCIError(CUI, id()) << "Unrecognized CUI mode:" << mode;
#else
        yCError(CUI) << "Unrecognized CUI mode:" << mode;
#endif
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
