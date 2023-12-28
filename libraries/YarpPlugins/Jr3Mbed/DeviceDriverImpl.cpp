// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <functional> // std::bind

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_ACK_TIMEOUT = 0.01; // [s]
constexpr auto DEFAULT_MONITOR_PERIOD = 0.1; // [s]
constexpr auto DEFAULT_FILTER = 0.0; // [Hz], 0.0 = no filter

// -----------------------------------------------------------------------------

bool Jr3Mbed::open(yarp::os::Searchable & config)
{
    if (!config.check("robotConfig") || !config.find("robotConfig").isBlob())
    {
        yCError(JR3M) << R"(Missing "robotConfig" property or not a blob)";
        return false;
    }

    const auto * robotConfig = *reinterpret_cast<const yarp::os::Property * const *>(config.find("robotConfig").asBlob());

    const auto & commonGroup = robotConfig->findGroup("common-jr3");
    yarp::os::Property jr3Group;

    if (!commonGroup.isNull())
    {
        yCDebugOnce(JR3M) << commonGroup.toString();
        jr3Group.fromString(commonGroup.toString());
    }

    jr3Group.fromString(config.toString(), false); // override common options

    canId = config.check("canId", yarp::os::Value(0), "CAN bus ID").asInt8(); // id-specific
    name = config.check("name", yarp::os::Value(""), "sensor name").asString(); // id-specific
    filter = jr3Group.check("filter", yarp::os::Value(DEFAULT_FILTER), "cutoff frequency for low-pass filter (Hertz)").asFloat64();

    if (canId <= 0)
    {
        yCError(JR3M) << "Illegal CAN ID:" << canId;
        return false;
    }

    if (filter < 0.0)
    {
        yCError(JR3M) << "Illegal filter value:" << filter;
        return false;
    }

    yarp::dev::DeviceDriver::setId("ID" + std::to_string(canId));

    auto ackTimeout = jr3Group.check("ackTimeout", yarp::os::Value(DEFAULT_ACK_TIMEOUT), "CAN acknowledge timeout (seconds)").asFloat64();

    if (ackTimeout <= 0.0)
    {
        yCIError(JR3M, id()) << R"(Illegal "ackTimeout" value:)" << ackTimeout;
        return false;
    }

    ackStateObserver = new TypedStateObserver<std::uint8_t[]>(ackTimeout);

    if (!config.check("fullScales", "full scales for each axis")) // id-specific
    {
        yCIError(JR3M, id()) << R"(Missing "fullScales" property)";
        return false;
    }

    if (jr3Group.check("fullScales", "list of full scales for each axis (3*N, 3*daNm)"))
    {
        const auto & fullScalesValue = jr3Group.find("fullScales");

        if (!fullScalesValue.isList() || fullScalesValue.asList()->size() != 6)
        {
            yCIError(JR3M, id()) << R"(The "fullScales" property must be a 6-element list of integers)";
            return false;
        }

        yCIInfo(JR3M, id()) << "Using full scales from configuration file:" << fullScalesValue.toString();

        const auto * fullScales = fullScalesValue.asList();

        for (int i = 0; i < 3; i++)
        {
            scales[i] = fullScales->get(i).asInt16() / static_cast<double>(FULL_SCALE);
        }

        for (int i = 3; i < 6; i++)
        {
            scales[i] = fullScales->get(i).asInt16() / (static_cast<double>(FULL_SCALE) * 10);
        }
    }
    else
    {
        yCIInfo(JR3M, id()) << R"(Missing "fullScales" property, will be queried from sensor on startup)";
        shouldQueryFullScales = true;
    }

    if (jr3Group.check("asyncPeriod", "period of asynchronous publishing mode (seconds)"))
    {
        asyncPeriod = jr3Group.find("asyncPeriod").asFloat64();
        yCIInfo(JR3M, id()) << "Asynchronous mode requested with period" << asyncPeriod << "[s]";

        if (asyncPeriod <= 0.0)
        {
            yCIError(JR3M, id()) << R"(Illegal "asyncPeriod" value:)" << asyncPeriod;
            return false;
        }

        mode = jr3_mode::ASYNC;
    }
    else
    {
        yCIInfo(JR3M, id()) << "Synchronous mode requested";
        mode = jr3_mode::SYNC;
    }

    auto monitorPeriod = jr3Group.check("monitorPeriod", yarp::os::Value(DEFAULT_MONITOR_PERIOD), "monitor thread period (seconds)").asFloat64();

    if (monitorPeriod <= 0.0)
    {
        yCIError(JR3M, id()) << R"(Illegal "monitorPeriod" value:)" << monitorPeriod;
        return false;
    }

    using namespace std::placeholders;
    monitorThread = new yarp::os::Timer(monitorPeriod, std::bind(&Jr3Mbed::monitorWorker, this, _1), false);

    // no more than 8 seconds since packets arrive at 8 KHz and the counter is only 16 bits wide (it overflows every 65536 frames)
    diagnosticsPeriod = config.check("diagnosticsPeriod", yarp::os::Value(0.0), "diagnostics period (seconds)").asFloat64();

    if (diagnosticsPeriod < 0.0)
    {
        yCIError(JR3M, id()) << R"(Illegal "diagnosticsPeriod" value:)" << diagnosticsPeriod;
        return false;
    }

    return monitorThread->start();
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::close()
{
    // we need to do this in finalize(), too, since the monitor thread could be
    // still requesting CAN transfers even after CAN RX/TX threads have been
    // closed in CanBusBroker::close()
    if (monitorThread && monitorThread->isRunning())
    {
        monitorThread->stop();
    }

    delete monitorThread;
    monitorThread = nullptr;

    delete ackStateObserver;
    ackStateObserver = nullptr;

    return true;
}

// -----------------------------------------------------------------------------
