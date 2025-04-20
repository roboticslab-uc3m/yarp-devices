// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <functional> // std::bind

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_MONITOR_PERIOD = 0.1; // [s]

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

    if (!parseParams(jr3Group))
    {
        yCIError(JR3M, id()) << "Could not parse parameters";
        return false;
    }

    if (m_canId <= 0 || m_canId > 127)
    {
        yCError(JR3M) << "Illegal CAN ID:" << m_canId;
        return false;
    }

    if (m_filter < 0.0 || m_filter > 655.35) // (2^16 - 1) / 100
    {
        yCError(JR3M) << "Illegal filter value:" << m_filter;
        return false;
    }

    yarp::dev::DeviceDriver::setId("ID" + std::to_string(m_canId));

    if (m_ackTimeout <= 0.0)
    {
        yCIError(JR3M, id()) << R"(Illegal "ackTimeout" value:)" << m_ackTimeout;
        return false;
    }

    ackStateObserver = new TypedStateObserver<std::uint8_t[]>(m_ackTimeout);

    if (!m_fullScales.empty())
    {
        if (m_fullScales.size() != 6)
        {
            yCIError(JR3M, id()) << R"(The "fullScales" property must be a 6-element list of integers)";
            return false;
        }

        yCIInfo(JR3M, id()) << "Using full scales from configuration file:" << m_fullScales;

        for (int i = 0; i < 3; i++)
        {
            scales[i] = m_fullScales[i] / static_cast<double>(FULL_SCALE);
        }

        for (int i = 3; i < 6; i++)
        {
            scales[i] = m_fullScales[i] / (static_cast<double>(FULL_SCALE) * 10);
        }
    }
    else
    {
        yCIInfo(JR3M, id()) << R"(Missing "fullScales" property, will be queried from sensor on startup)";
        shouldQueryFullScales = true;
    }

    if (m_asyncPeriod > 0.0)
    {
        yCIInfo(JR3M, id()) << "Asynchronous mode requested with period" << m_asyncPeriod << "[s]";
        mode = jr3_mode::ASYNC;
    }
    else
    {
        yCIInfo(JR3M, id()) << "Synchronous mode requested";
        mode = jr3_mode::SYNC;
    }

    if (m_monitorPeriod <= 0.0)
    {
        yCIError(JR3M, id()) << R"(Illegal "monitorPeriod" value:)" << m_monitorPeriod;
        return false;
    }

    using namespace std::placeholders;
    monitorThread = new yarp::os::Timer(m_monitorPeriod, std::bind(&Jr3Mbed::monitorWorker, this, _1), false);

    // no more than 8 seconds since packets arrive at 8 KHz and the counter is only 16 bits wide (it overflows every 65536 frames)

    if (m_diagnosticsPeriod < 0.0)
    {
        yCIError(JR3M, id()) << R"(Illegal "diagnosticsPeriod" value:)" << m_diagnosticsPeriod;
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
