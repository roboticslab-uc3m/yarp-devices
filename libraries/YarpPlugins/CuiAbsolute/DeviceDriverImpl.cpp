// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <cmath>

#include <yarp/os/Time.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CuiAbsolute::open(yarp::os::Searchable & config)
{
    if (!config.check("robotConfig") || !config.find("robotConfig").isBlob())
    {
        CD_ERROR("Missing \"robotConfig\" property or not a blob.\n");
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

    CD_DEBUG("%s\n", cuiGroup.toString().c_str());

    canId = config.check("canId", yarp::os::Value(0), "CAN bus ID").asInt8(); // id-specific
    reverse = cuiGroup.check("reverse", yarp::os::Value(false), "reverse").asBool();
    timeout = cuiGroup.check("timeout", yarp::os::Value(DEFAULT_TIMEOUT), "timeout (seconds)").asFloat64();
    maxRetries = cuiGroup.check("maxRetries", yarp::os::Value(DEFAULT_MAX_RETRIES), "max retries on timeout").asFloat64();

    if (timeout <= 0.0)
    {
        CD_ERROR("Illegal CUI timeout value: %f.\n", timeout);
        return false;
    }

    if (!cuiGroup.check("mode", "publish mode [push|pull]"))
    {
        CD_ERROR("Missing \"mode\" property.\n");
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
        CD_ERROR("Unrecognized CUI mode: %s.\n", mode.c_str());
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
