// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_TIMEOUT = 0.25; // [s]
constexpr auto DEFAULT_MAX_RETRIES = 10;
constexpr auto DEFAULT_FILTER = 0.0; // [Hz]

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
    auto timeout = jr3Group.check("timeout", yarp::os::Value(DEFAULT_TIMEOUT), "timeout [s]").asFloat64();
    auto maxRetries = jr3Group.check("maxRetries", yarp::os::Value(DEFAULT_MAX_RETRIES), "max retries on timeout").asFloat64();

    if (canId <= 0)
    {
        yCError(JR3M) << "Illegal CAN ID:" << canId;
        return false;
    }

    yarp::dev::DeviceDriver::setId("ID" + std::to_string(canId));

    if (timeout <= 0.0)
    {
        yCIError(JR3M, id()) << "Illegal timeout value:" << timeout;
        return false;
    }

    if (!jr3Group.check("mode", "publish mode [sync|async]"))
    {
        yCIError(JR3M, id()) << "Missing \"mode\" property";
        return false;
    }

    std::string mode = jr3Group.find("mode").asString();

    auto filter = config.check("filter", yarp::os::Value(DEFAULT_FILTER), "cutoff frequency for low-pass filter [Hz]").asFloat64();

    return true;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::close()
{
    return true;
}

// -----------------------------------------------------------------------------
