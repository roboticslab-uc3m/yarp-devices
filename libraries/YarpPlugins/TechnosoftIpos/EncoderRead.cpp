// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EncoderRead.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

EncoderRead::EncoderRead(double samplingPeriod)
    : samplingFreq(1.0 / samplingPeriod),
      lastPosition(0),
      lastSpeed(0.0),
      lastAcceleration(0.0)
{
    lastStamp.update();
}

// -----------------------------------------------------------------------------

void EncoderRead::update(std::int32_t newPos)
{
    std::lock_guard guard(encoderMutex);

    const double lastTime = lastStamp.getTime();
    const double nextToLastPosition = lastPosition;
    const double nextToLastSpeed = lastSpeed;

    lastStamp.update();
    const double samples = (lastStamp.getTime() - lastTime) * samplingFreq;

    lastPosition = newPos;
    lastSpeed = (lastPosition - nextToLastPosition) / samples;
    lastAcceleration = (lastSpeed - nextToLastSpeed) / samples;
}

// -----------------------------------------------------------------------------

void EncoderRead::reset(std::int32_t pos)
{
    std::lock_guard guard(encoderMutex);
    lastPosition = pos;
    lastSpeed = lastAcceleration = 0.0;
    lastStamp.update();
}

// -----------------------------------------------------------------------------

std::int32_t EncoderRead::queryPosition() const
{
    std::lock_guard guard(encoderMutex);
    return lastPosition;
}

// -----------------------------------------------------------------------------

double EncoderRead::querySpeed() const
{
    std::lock_guard guard(encoderMutex);
    return lastSpeed;
}

// -----------------------------------------------------------------------------

double EncoderRead::queryAcceleration() const
{
    std::lock_guard guard(encoderMutex);
    return lastAcceleration;
}

// -----------------------------------------------------------------------------

double EncoderRead::queryTime() const
{
    std::lock_guard guard(encoderMutex);
    return lastStamp.getTime();
}

// -----------------------------------------------------------------------------
