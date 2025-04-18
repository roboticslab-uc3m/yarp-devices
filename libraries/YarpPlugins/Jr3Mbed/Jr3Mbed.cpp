// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <cstdint>
#include <cstring> // std::memcpy

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool Jr3Mbed::performRequest(const std::string & cmd, const can_message & msg, std::uint8_t * response, bool quiet)
{
    yCIInfo(quiet ? JR3M_QUIET : JR3M, id()) << "Sending" << cmd << "command";

    if (!sender || !sender->prepareMessage(msg))
    {
        yCIWarning(quiet ? JR3M_QUIET : JR3M, id()) << "Unable to register" << cmd << "command";
        return false;
    }

    if (!ackStateObserver->await(response))
    {
        yCIWarning(quiet ? JR3M_QUIET : JR3M, id()) << "Command" << cmd << "timed out";
        return false;
    }

    if (response[0] != static_cast<unsigned int>(jr3_state::READY))
    {
        yCIError(JR3M, id()) << "Sensor is in error state";
        status = yarp::dev::MAS_ERROR;
        // avoid calling this repeatedly from the monitor thread; no concurrency issues here
        // since all requests are handled either from the monitor thread or from finalize()
        sender->reportAvailability(false, m_canId);
        return false;
    }

    // all queries happen during initialization or shutdown, i.e. the usual transition involved
    // here is from UNKNOWN/ERROR to WAITING (not during normal operation, besides stopping)
    status = yarp::dev::MAS_WAITING_FOR_FIRST_READ;
    return true;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::sendStartSyncCommand(double _filter)
{
    std::string cmd = "start sync";
    std::uint16_t filter = _filter * 100;

    unsigned char data[sizeof(filter)];
    std::memcpy(data, &filter, sizeof(filter));

    can_message msg {getCommandId(can_ops::START_SYNC), sizeof(filter), data};
    std::uint8_t response[1];
    return performRequest(cmd, msg, response);
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::sendStartAsyncCommand(double _filter, double _period)
{
    std::string cmd = "start async";
    std::uint16_t filter = _filter * 100;
    std::uint32_t period = _period * 1e6;

    unsigned char data[sizeof(filter) + sizeof(period)];
    std::memcpy(data, &filter, sizeof(filter));
    std::memcpy(data + sizeof(filter), &period, sizeof(period));

    can_message msg {getCommandId(can_ops::START_ASYNC), sizeof(filter) + sizeof(period), data};
    std::uint8_t response[1];
    return performRequest(cmd, msg, response);
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::sendCommand(const std::string & cmd, can_ops op)
{
    can_message msg {getCommandId(op), 0, nullptr};
    std::uint8_t response[1];
    return performRequest(cmd, msg, response);
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::ping()
{
    can_message msg {getCommandId(can_ops::GET_STATE), 0, nullptr};
    std::uint8_t response[1];
    return performRequest("ping", msg, response, true);
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::queryFullScales()
{
    std::uint8_t response[7];
    std::uint16_t value;

    can_message msgForces {getCommandId(can_ops::GET_FS_FORCES), 0, nullptr};

    if (!performRequest("get full scales (forces)", msgForces, response))
    {
        return false;
    }

    for (int i = 0; i < 3; i++)
    {
        std::memcpy(&value, response + 1 + (2 * i), sizeof(value));
        scales[i] = value / static_cast<double>(FULL_SCALE);
    }

    can_message msgMoments {getCommandId(can_ops::GET_FS_MOMENTS), 0, nullptr};

    if (!performRequest("get full scales (forces)", msgMoments, response))
    {
        return false;
    }

    for (int i = 0; i < 3; i++)
    {
        std::memcpy(&value, response + 1 + (2 * i), sizeof(value));
        scales[i + 3] = value / (static_cast<double>(FULL_SCALE) * 10);
    }

    shouldQueryFullScales = false;
    return true;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::monitorWorker(const yarp::os::YarpTimerEvent & event)
{
    if (isBooting.exchange(false)) // enter the condition if true, then set to false
    {
        status = yarp::dev::MAS_WAITING_FOR_FIRST_READ;

        if (sender)
        {
            sender->reportAvailability(true, m_canId);
        }

        if (!initialize() && sender)
        {
            sender->reportAvailability(false, m_canId);
        }

        return true;
    }

    mtx.lock();
    auto elapsed = event.currentReal - timestamp;
    auto counter = frameCounter;
    mtx.unlock();

    if (elapsed < event.lastDuration) // we have received data in time
    {
        // either timeout or awaiting first read
        if (status.exchange(yarp::dev::MAS_OK) != yarp::dev::MAS_OK)
        {
            yCIInfo(JR3M, id()) << "Sensor is responding";

            if (sender)
            {
                sender->reportAvailability(true, m_canId);
            }

            lastDiagnosticsTimestamp = event.currentReal;
            lastFrameCounter = counter;
        }
        else if (m_diagnosticsPeriod != 0.0 && event.currentReal - lastDiagnosticsTimestamp > m_diagnosticsPeriod)
        {
            auto diff = lastFrameCounter > counter ? counter + (0xFFFF - lastFrameCounter) : counter - lastFrameCounter;
            auto rate = diff / (event.currentReal - lastDiagnosticsTimestamp);
            yCIDebug(JR3M, id()) << "Frame rate:" << rate << "Hz," << diff << "packets";
            lastDiagnosticsTimestamp = event.currentReal;
            lastFrameCounter = counter;
        }
    }
    else if (status == yarp::dev::MAS_ERROR)
    {
        // reportAvailability() has already been called from performRequest()
    }
    else if (status == yarp::dev::MAS_WAITING_FOR_FIRST_READ)
    {
        // nothing to do here
    }
    else if (status.exchange(yarp::dev::MAS_TIMEOUT) == yarp::dev::MAS_OK)
    {
        yCIWarning(JR3M, id()) << "Sensor has timed out, last data was received" << elapsed << "seconds ago";

        if (sender)
        {
            sender->reportAvailability(false, m_canId);
        }
    }
    else
    {
        // still in timeout state
    }

    return true;
}

// -----------------------------------------------------------------------------
