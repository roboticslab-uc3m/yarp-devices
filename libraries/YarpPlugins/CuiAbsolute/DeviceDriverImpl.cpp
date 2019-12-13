// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <cmath>

#include <yarp/os/Time.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CuiAbsolute::open(yarp::os::Searchable& config)
{
    CD_DEBUG("%s\n", config.toString().c_str());

    canId = config.check("canId", yarp::os::Value(0), "CAN bus ID").asInt8();
    reverse = config.check("reverse", yarp::os::Value(false), "reverse").asBool();
    timeout = config.check("timeout", yarp::os::Value(DEFAULT_TIMEOUT), "timeout (seconds)").asFloat64();
    maxRetries = config.check("maxRetries", yarp::os::Value(DEFAULT_MAX_RETRIES), "max retries on timeout").asFloat64();

    if (timeout <= 0.0)
    {
        CD_ERROR("Illegal CUI timeout value: %f.\n", timeout);
        return false;
    }

    if (!config.check("mode", "publish mode [push|pull]"))
    {
        CD_ERROR("Missing \"mode\" property.\n");
        return false;
    }

    std::string mode = config.find("mode").asString();

    if (mode == "push")
    {
        pushStateObserver = new StateObserver(timeout);
        cuiMode = CuiMode::PUSH;
        pushDelay = config.check("pushDelay", yarp::os::Value(0), "Cui push mode delay [0-255]").asInt8();
    }
    else if (mode == "pull")
    {
        pollStateObserver = new TypedStateObserver<encoder_t>(timeout);
        cuiMode = CuiMode::PULL;
    }
    else
    {
        CD_ERROR("Unrecognized CUI mode: %s.\n", mode.c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::close()
{
    switch (cuiMode)
    {
    case CuiMode::PUSH:
        delete pushStateObserver;
        break;
    case CuiMode::PULL:
        delete pollStateObserver;
        break;
    }

    return true;
}

// -----------------------------------------------------------------------------
