// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Mbed.hpp"

#include <cstdint>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool Jr3Mbed::performRequest(const std::string & cmd, const can_message & msg)
{
    if (!sender || !sender->prepareMessage(msg))
    {
        yCIWarning(JR3M, id()) << "Unable to register" << cmd << "command";
        return false;
    }

    if (!ackStateObserver->await())
    {
        yCIWarning(JR3M, id()) << "Command" << cmd << "timed out";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::sendStartSyncCommand(double _filter)
{
    std::string cmd = "start sync";
    std::uint16_t filter = _filter * 10;
    yCIInfo(JR3M, id()) << "Sending" << cmd << "command with filter" << filter * 0.1 << "Hz";

    unsigned char data[sizeof(filter)];
    std::memcpy(data, &filter, sizeof(filter));

    can_message msg {canId + (JR3_START_SYNC << 7), sizeof(filter), data};
    return performRequest(cmd, msg);
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::sendStartAsyncCommand(double _filter, double _period)
{
    std::string cmd = "start async";
    std::uint16_t filter = _filter * 10;
    std::uint32_t period = _period * 1e6;
    yCIInfo(JR3M, id()) << "Sending" << cmd << "command with filter" << filter * 0.1 << "Hz and period" << period << "us";

    unsigned char data[sizeof(filter) + sizeof(period)];
    std::memcpy(data, &filter, sizeof(filter));
    std::memcpy(data + sizeof(filter), &period, sizeof(period));

    can_message msg {canId + (JR3_START_ASYNC << 7), sizeof(filter) + sizeof(period), data};
    return performRequest(cmd, msg);
}

// -----------------------------------------------------------------------------

bool Jr3Mbed::sendStopCommand()
{
    std::string cmd = "stop";
    yCIInfo(JR3M, id()) << "Sending" << cmd << "command";
    can_message msg {canId + (JR3_STOP << 7), 0, nullptr};
    return performRequest(cmd, msg);
}

// -----------------------------------------------------------------------------
