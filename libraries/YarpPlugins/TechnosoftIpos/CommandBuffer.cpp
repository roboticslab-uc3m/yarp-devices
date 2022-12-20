// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CommandBuffer.hpp"

#include <algorithm> // std::max

#include <yarp/os/SystemClock.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

void CommandBuffer::accept(double command)
{
    std::lock_guard lock(mutex);
    double now = yarp::os::SystemClock::nowSystem();
    commandPeriod = now - commandTimestamp;
    commandTimestamp = now;
    storedCommand = command;
}

// -----------------------------------------------------------------------------

double CommandBuffer::interpolate()
{
    std::lock_guard lock(mutex);

    double now = yarp::os::SystemClock::nowSystem();
    double nextExpectedCommandTimestamp = commandTimestamp + commandPeriod;
    double interpolationPeriod = now - interpolationTimestamp;

    if (now >= nextExpectedCommandTimestamp || 1.1 * interpolationPeriod >= commandPeriod)
    {
        // either the next command was skipped, it didn't arrive on time or perhaps periods
        // were meant to match, so just return the last stored command
        interpolationResult = storedCommand;
    }
    else
    {
        // using `std::max` in order to soften the interpolated slope and thus avoid overshoot
        double interval = std::max(nextExpectedCommandTimestamp - interpolationTimestamp, interpolationPeriod);

        // having y = f(t): f(t+1) = f(t) + t * (delta_y / delta_t)
        interpolationResult += interpolationPeriod * (storedCommand - interpolationResult) / interval;
    }

    interpolationTimestamp = now;
    return interpolationResult;
}

// -----------------------------------------------------------------------------

double CommandBuffer::getStoredCommand(double * timestamp) const
{
    std::lock_guard lock(mutex);

    if (timestamp)
    {
        *timestamp = commandTimestamp;
    }

    return storedCommand;
}

// -----------------------------------------------------------------------------

void CommandBuffer::reset(double initialCommand)
{
    std::lock_guard lock(mutex);
    storedCommand = interpolationResult = initialCommand;
    commandPeriod = 0.0;
    commandTimestamp = interpolationTimestamp = yarp::os::SystemClock::nowSystem();
}

// -----------------------------------------------------------------------------
