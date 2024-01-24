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
    yarp::os::Property cuiOptions;
    cuiOptions.fromString(config.findGroup("common").toString());
    cuiOptions.fromString(config.toString(), false); // override common options

    canId = config.check("canId", yarp::os::Value(0), "CAN bus ID").asInt8(); // id-specific, don't override this
    reverse = cuiOptions.check("reverse", yarp::os::Value(false), "reverse").asBool();
    timeout = cuiOptions.check("timeout", yarp::os::Value(DEFAULT_TIMEOUT), "timeout (seconds)").asFloat64();
    maxRetries = cuiOptions.check("maxRetries", yarp::os::Value(DEFAULT_MAX_RETRIES), "max retries on timeout").asFloat64();

    if (canId <= 0 || canId > 127)
    {
        yCError(CUI) << "Illegal CAN ID:" << canId;
        return false;
    }

    if (timeout <= 0.0)
    {
        yCIError(CUI, id()) << "Illegal CUI timeout value:" << timeout;
        return false;
    }

    if (!cuiOptions.check("mode", "publish mode [push|pull]"))
    {
        yCIError(CUI, id()) << "Missing \"mode\" property";
        return false;
    }

    std::string mode = cuiOptions.find("mode").asString();

    if (mode == "push")
    {
        pushStateObserver = new StateObserver(timeout);
        cuiMode = CuiMode::PUSH;
        pushDelay = cuiOptions.check("pushDelay", yarp::os::Value(0), "Cui push mode delay [0-255]").asInt8();
    }
    else if (mode == "pull")
    {
        pollStateObserver = new TypedStateObserver<encoder_t>(timeout);
        cuiMode = CuiMode::PULL;
    }
    else
    {
        yCIError(CUI, id()) << "Unrecognized CUI mode:" << mode;
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
