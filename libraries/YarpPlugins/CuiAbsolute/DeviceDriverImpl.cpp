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
    timeout = config.check("timeout", yarp::os::Value(DEFAULT_TIMEOUT), "Cui timeout (seconds)").asFloat64();

    if (timeout <= 0.0)
    {
        CD_ERROR("Illegal CUI timeout value: %f.\n", timeout);
        return false;
    }

    std::string mode = config.check("mode", yarp::os::Value(DEFAULT_MODE), "Cui mode [push|pull]").asString();

    if (mode == "push")
    {
        pushStateObserver = new StateObserver(timeout);
        cuiMode = CuiMode::PUSH;
        pushDelay = config.check("pushDelay", yarp::os::Value(0), "Cui push mode delay [0-255]").asInt8();
    }
    else if (mode == "pull")
    {
        pollStateObserver = new TypedStateObserver<double>(timeout);
        cuiMode = CuiMode::PULL;
    }
    else
    {
        CD_ERROR("Unrecognized CUI mode: %s.\n", mode.c_str());
        return false;
    }

    CD_SUCCESS("Created CuiAbsolute with canId %d.\n", canId);
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
