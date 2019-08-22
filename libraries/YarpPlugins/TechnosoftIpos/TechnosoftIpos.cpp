// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include "CanUtils.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::sendLinearInterpolationTarget()
{
    return can->rpdo(3)->write<uint64_t>(linInterpBuffer->makeDataRecord());
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::sendLinearInterpolationStart()
{
    return can->rpdo(1)->write<uint16_t>(0x001F);
}

// -----------------------------------------------------------------------------

roboticslab::EncoderRead::EncoderRead(double initialPos)
    : lastPosition(initialPos),
      nextToLastPosition(initialPos),
      lastSpeed(0.0),
      nextToLastSpeed(0.0),
      lastAcceleration(0.0)
{
    lastStamp.update();
}

// -----------------------------------------------------------------------------

void roboticslab::EncoderRead::update(double newPos, double newTime)
{
    std::lock_guard<std::mutex> guard(encoderMutex);

    const double lastTime = lastStamp.getTime();

    nextToLastPosition = lastPosition;
    nextToLastSpeed = lastSpeed;

    if (newTime)
    {
        lastStamp.update(newTime);
    }
    else
    {
        lastStamp.update();
    }

    double dt = lastStamp.getTime() - lastTime;

    lastPosition = newPos;
    lastSpeed = (lastPosition - nextToLastPosition) / dt;
    lastAcceleration = (lastSpeed - nextToLastSpeed) / dt;
}

// -----------------------------------------------------------------------------

double roboticslab::EncoderRead::queryPosition() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastPosition;
}

// -----------------------------------------------------------------------------

double roboticslab::EncoderRead::querySpeed() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastSpeed;
}

// -----------------------------------------------------------------------------

double roboticslab::EncoderRead::queryAcceleration() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastAcceleration;
}

// -----------------------------------------------------------------------------

double roboticslab::EncoderRead::queryTime() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastStamp.getTime();
}

// -----------------------------------------------------------------------------
