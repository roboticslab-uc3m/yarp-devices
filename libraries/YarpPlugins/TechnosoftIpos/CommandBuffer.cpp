// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CommandBuffer.hpp"

#include <yarp/os/SystemClock.h>

using namespace roboticslab;

constexpr double COMMAND_TIMEOUT = 1.0; // seconds

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

    const double now = yarp::os::SystemClock::nowSystem();
    const double nextExpectedCommandTimestamp = commandTimestamp + commandPeriod;
    const double interpolationPeriod = now - interpolationTimestamp;

    if (now > nextExpectedCommandTimestamp || commandPeriod > COMMAND_TIMEOUT)
    {
        // cond. 1: the next command was skipped, it didn't arrive on time or none was received yet
        // cond. 2: the last command was received too long ago, perhaps it was a single step command?
        // in either case, we don't interpolate and just process the last received command (again)
        interpolationResult = storedCommand;
    }
    else
    {
        // note that `interpolationTimestamp` is equal to the `now` value of the previous iteration,
        // hence the reaching of `storedCommand` will be delayed (resulting in a softer slope)
        double slope = (storedCommand - interpolationResult) / (nextExpectedCommandTimestamp - interpolationTimestamp);

        // having y = f(t): f(t+T) = f(t) + T * (delta_y / delta_t)
        interpolationResult += interpolationPeriod * slope;
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
