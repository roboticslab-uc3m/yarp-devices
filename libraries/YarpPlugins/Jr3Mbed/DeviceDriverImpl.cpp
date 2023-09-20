// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto FULL_SCALE = 16384; // 2^14

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

    yarp::dev::DeviceDriver::setId("ID" + std::to_string(canId));

    auto ackTimeout = jr3Group.check("ackTimeout", yarp::os::Value(DEFAULT_ACK_TIMEOUT), "CAN acknowledge timeout (seconds)").asFloat64();

    if (ackTimeout <= 0.0)
    {
        yCIError(JR3M, id()) << R"(Illegal "ackTimeout" value:)" << ackTimeout;
        return false;
    }

    ackStateObserver = new StateObserver(ackTimeout);

    if (!config.check("fullScales", "full scales for each axis")) // id-specific
    {
        yCIError(JR3M, id()) << R"(Missing "fullScales" property)";
        return false;
    }

    const auto & fullScalesValue = jr3Group.find("fullScales");

    if (!fullScalesValue.isList() || fullScalesValue.asList()->size() != 6)
    {
        yCIError(JR3M, id()) << R"(The "fullScales" property must be a 6-element list)";
        return false;
    }

    const auto * fullScales = fullScalesValue.asList();

    for (int i = 0; i < 3; i++)
    {
        forceScales.push_back(fullScales->get(i).asFloat64() / FULL_SCALE);
    }

    for (int i = 3; i < 6; i++)
    {
        momentScales.push_back(fullScales->get(i).asFloat64() / (FULL_SCALE * 10));
    }

    if (jr3Group.check("asyncPeriod", "period of asynchronous publishing mode (seconds)"))
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

    auto monitorPeriod = jr3Group.check("monitorPeriod", yarp::os::Value(DEFAULT_MONITOR_PERIOD), "monitor thread period (seconds)").asFloat64();

    if (monitorPeriod <= 0.0)
    {
        yCIError(JR3M, id()) << R"(Illegal "monitorPeriod" value:)" << monitorPeriod;
        return false;
    }

    status = yarp::dev::MAS_WAITING_FOR_FIRST_READ;

    monitorThread = new yarp::os::Timer(yarp::os::TimerSettings(monitorPeriod), [this](const auto & event)
        {
            std::lock_guard lock(rxMutex);

            double elapsed = event.currentReal - timestamp;

            if (elapsed < event.lastDuration)
            {
                if (status != yarp::dev::MAS_OK) // either timeout or awaiting first read
                {
                    yCIInfo(JR3M, id()) << "Sensor is responding";
                }

                status = yarp::dev::MAS_OK;
            }
            else if (status == yarp::dev::MAS_OK) // timeout!
            {
                yCIWarning(JR3M, id()) << "Sensor has timed out, last data was received" << elapsed << "seconds ago";
                status = yarp::dev::MAS_TIMEOUT;
            }
            else
            {
                // still waiting for first read
            }

            return true;
        }, false);

    return true;
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
