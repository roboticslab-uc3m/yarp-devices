// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StateVariables.hpp"

#include <cmath>

#include <ColorDebug.h>

using namespace roboticslab;

namespace
{
    // return -1 for negative numbers, +1 for positive numbers, 0 for zero
    // https://stackoverflow.com/a/4609795
    template<typename T>
    inline int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }
};

// -----------------------------------------------------------------------------

EncoderRead::EncoderRead(double initialPos)
    : lastPosition(initialPos),
      nextToLastPosition(initialPos),
      lastSpeed(0.0),
      nextToLastSpeed(0.0),
      lastAcceleration(0.0)
{
    lastStamp.update();
}

// -----------------------------------------------------------------------------

void EncoderRead::reset(double pos)
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    lastPosition = nextToLastPosition = pos;
    lastSpeed = nextToLastSpeed = lastAcceleration = 0.0;
}

// -----------------------------------------------------------------------------

void EncoderRead::update(double newPos, double newTime)
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

double EncoderRead::queryPosition() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastPosition;
}

// -----------------------------------------------------------------------------

double EncoderRead::querySpeed() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastSpeed;
}

// -----------------------------------------------------------------------------

double EncoderRead::queryAcceleration() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastAcceleration;
}

// -----------------------------------------------------------------------------

double EncoderRead::queryTime() const
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    return lastStamp.getTime();
}

// -----------------------------------------------------------------------------

// TODO: add mutex guards?

StateVariables::StateVariables()
    : lastEncoderRead(0.0),
      controlMode(0),
      drivePeakCurrent(0.0),
      maxVel(0.0),
      tr(0.0),
      k(0.0),
      min(0.0),
      max(0.0),
      refSpeed(0.0),
      refAcceleration(0.0),
      encoderPulses(0),
      pulsesPerSample(0)
{}

// -----------------------------------------------------------------------------

bool StateVariables::validateInitialState()
{
    if (controlMode == 0)
    {
        CD_WARNING("Illegal initial control mode.\n");
        return false;
    }

    if (drivePeakCurrent <= 0.0)
    {
        CD_WARNING("Illegal drive peak current: %f.\n", drivePeakCurrent);
        return false;
    }

    if (maxVel <= 0.0)
    {
        CD_WARNING("Illegal maximum velocity: %f.\n", maxVel);
        return false;
    }

    if (tr <= 0.0)
    {
        CD_WARNING("Illegal transmission ratio: %f.\n", tr);
        return false;
    }

    if (k <= 0.0)
    {
        CD_WARNING("Illegal motor constant: %f.\n", k);
        return false;
    }

    if (refSpeed <= 0.0)
    {
        CD_WARNING("Illegal reference speed: %f.\n", refSpeed);
        return false;
    }

    if (refAcceleration <= 0.0)
    {
        CD_WARNING("Illegal reference acceleration: %f.\n", refAcceleration);
        return false;
    }

    if (encoderPulses <= 0)
    {
        CD_WARNING("Illegal encoder pulses per revolution: %d.\n", encoderPulses);
        return false;
    }

    if (pulsesPerSample <= 0)
    {
        CD_WARNING("Illegal pulses per sample: %d.\n", pulsesPerSample);
        return false;
    }

    if (refSpeed > maxVel)
    {
        CD_WARNING("Reference speed greater than maximum velocity: %f > %f.\n", refSpeed, maxVel);
        return false;
    }

    if (min >= max)
    {
        CD_WARNING("Illegal joint limits (min, max): %f >= %f.\n", min, max);
        return false;
    }
}

// -----------------------------------------------------------------------------

std::int32_t StateVariables::degreesToInternalUnits(double value, int derivativeOrder)
{
    return value * tr * (encoderPulses / 360.0) * std::pow(1.0 / pulsesPerSample, derivativeOrder);
}

// -----------------------------------------------------------------------------

double StateVariables::internalUnitsToDegrees(std::int32_t value, int derivativeOrder)
{
    return value / (tr * (encoderPulses / 360.0) * std::pow(1.0 / pulsesPerSample, derivativeOrder));
}

// -----------------------------------------------------------------------------

std::int16_t StateVariables::currentToInternalUnits(double value)
{
    return value * sgn(tr) * 65520.0 / (2.0 * drivePeakCurrent);
}

// -----------------------------------------------------------------------------

double StateVariables::internalUnitsToCurrent(std::int16_t value)
{
    return value * sgn(tr) * 2.0 * drivePeakCurrent / 65520.0;
}

// -----------------------------------------------------------------------------

double StateVariables::internalUnitsToPeakCurrent(std::int16_t value)
{
    return 2.0 * drivePeakCurrent * (32767.0 - value) / 65520.0;
}

// -----------------------------------------------------------------------------

double StateVariables::currentToTorque(double current)
{
    return current * std::abs(tr) * k;
}

// -----------------------------------------------------------------------------

double StateVariables::torqueToCurrent(double torque)
{
    return torque / (std::abs(tr) * k);
}

// -----------------------------------------------------------------------------
