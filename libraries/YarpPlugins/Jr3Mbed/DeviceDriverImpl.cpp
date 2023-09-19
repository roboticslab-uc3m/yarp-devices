// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_TIMEOUT = 0.1; // [s]
constexpr auto DEFAULT_FILTER = 0.0; // [Hz], 0.0 = no filter

// -----------------------------------------------------------------------------

bool Jr3Mbed::open(yarp::os::Searchable & config)
{
    if (!config.check("robotConfig") || !config.find("robotConfig").isBlob())
    {
        yCError(JR3M) << "Missing \"robotConfig\" property or not a blob";
        return false;
    }

    const auto * robotConfig = *reinterpret_cast<const yarp::os::Property * const *>(config.find("robotConfig").asBlob());

    yarp::os::Bottle & commonGroup = robotConfig->findGroup("common-jr3");
    yarp::os::Property jr3Group;

    if (!commonGroup.isNull())
    {
        yCDebugOnce(JR3M) << commonGroup.toString();
        jr3Group.fromString(commonGroup.toString());
    }

    jr3Group.fromString(config.toString(), false); // override common options

    canId = config.check("canId", yarp::os::Value(0), "CAN bus ID").asInt8(); // id-specific
    filter = config.check("filter", yarp::os::Value(DEFAULT_FILTER), "cutoff frequency for low-pass filter [Hz]").asFloat64();
    auto timeout = jr3Group.check("timeout", yarp::os::Value(DEFAULT_TIMEOUT), "CAN acknowledge timeout [s]").asFloat64();

    if (canId <= 0)
    {
        yCError(JR3M) << "Illegal CAN ID:" << canId;
        return false;
    }

    yarp::dev::DeviceDriver::setId("ID" + std::to_string(canId));

    if (timeout <= 0.0)
    {
        yCIError(JR3M, id()) << R"(Illegal "timeout" value:)" << timeout;
        return false;
    }

    ackStateObserver = new StateObserver(timeout);

    if (jr3Group.check("asyncPeriod", "period of asynchronous publishing mode [s]"))
    {
        yCIInfo(JR3M, id()) << "Asynchronous mode requested";
        asyncPeriod = jr3Group.find("asyncPeriod").asFloat64();

        if (asyncPeriod <= 0.0)
        {
            yCIError(JR3M, id()) << R"(Illegal "asyncPeriod" value:)" << asyncPeriod;
            return false;
        }

        mode = ASYNC;
    }
    else
    {
        yCIInfo(JR3M, id()) << "Synchronous mode requested";
        mode = SYNC;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::close()
{
    if (ackStateObserver)
    {
        delete ackStateObserver;
        ackStateObserver = nullptr;
    }

    return true;
}

// -----------------------------------------------------------------------------
