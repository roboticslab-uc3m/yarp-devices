// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <cstdint>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool Jr3Mbed::performRequest(const std::string & cmd, const can_message & msg, bool quiet)
{
    if (!sender || !sender->prepareMessage(msg))
    {
        yCIWarning((quiet ? JR3M_QUIET : JR3M), id()) << "Unable to register" << cmd << "command";
        return false;
    }

    std::uint8_t response;

    if (!ackStateObserver->await(&response))
    {
        yCIWarning((quiet ? JR3M_QUIET : JR3M), id()) << "Command" << cmd << "timed out";
        return false;
    }

    if (response != static_cast<unsigned int>(jr3_state::READY))
    {
        yCIError(JR3M, id()) << "Sensor is in error state";
        status = yarp::dev::MAS_ERROR;
        // avoid calling this repeatedly from the monitor thread; no concurrency issues here
        // since all requests are handled either from the monitor thread or from finalize()
        sender->reportAvailability(false, canId);
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
    yCIInfo(JR3M, id()) << "Sending" << cmd << "command with filter" << filter * 0.01 << "Hz";

    unsigned char data[sizeof(filter)];
    std::memcpy(data, &filter, sizeof(filter));

    can_message msg {getCommandId(can_ops::START_SYNC), sizeof(filter), data};
    return performRequest(cmd, msg);
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::sendStartAsyncCommand(double _filter, double _period)
{
    std::string cmd = "start async";
    std::uint16_t filter = _filter * 100;
    std::uint32_t period = _period * 1e6;
    yCIInfo(JR3M, id()) << "Sending" << cmd << "command with filter" << filter * 0.01 << "Hz and period" << period * 1e-3 << "ms";

    unsigned char data[sizeof(filter) + sizeof(period)];
    std::memcpy(data, &filter, sizeof(filter));
    std::memcpy(data + sizeof(filter), &period, sizeof(period));

    can_message msg {getCommandId(can_ops::START_ASYNC), sizeof(filter) + sizeof(period), data};
    return performRequest(cmd, msg);
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::sendCommand(const std::string & cmd, can_ops op)
{
    yCIInfo(JR3M, id()) << "Sending" << cmd << "command";
    can_message msg {getCommandId(op), 0, nullptr};
    return performRequest(cmd, msg);
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::ping()
{
    return performRequest("ping", {getCommandId(can_ops::GET_STATE), 0, nullptr}, true);
}

// -----------------------------------------------------------------------------
