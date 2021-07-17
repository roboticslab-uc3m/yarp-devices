// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <cmath>

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
        cuiGroup.fromString(commonGroup.toString());
    }

    cuiGroup.fromString(config.toString(), false); // override common options

    yCDebug(CUI) << "Config:" << cuiGroup.toString();

    canId = config.check("canId", yarp::os::Value(0), "CAN bus ID").asInt8(); // id-specific
    reverse = cuiGroup.check("reverse", yarp::os::Value(false), "reverse").asBool();
    timeout = cuiGroup.check("timeout", yarp::os::Value(DEFAULT_TIMEOUT), "timeout (seconds)").asFloat64();
    maxRetries = cuiGroup.check("maxRetries", yarp::os::Value(DEFAULT_MAX_RETRIES), "max retries on timeout").asFloat64();

    if (timeout <= 0.0)
    {
        yCError(CUI) << "Illegal CUI timeout value:" << timeout;
        return false;
    }

    if (!cuiGroup.check("mode", "publish mode [push|pull]"))
    {
        yCError(CUI) << "Missing \"mode\" property";
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
        yCError(CUI) << "Unrecognized CUI mode:" << mode;
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
