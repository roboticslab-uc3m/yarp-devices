// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StateVariables.hpp"

#include <cmath>

#include <yarp/os/Vocab.h>
#include <yarp/dev/IAxisInfo.h>

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

EncoderRead::EncoderRead()
    : EncoderRead(0.0)
{ }

// -----------------------------------------------------------------------------

EncoderRead::EncoderRead(std::int32_t initialPos)
    : lastPosition(initialPos),
      nextToLastPosition(initialPos),
      lastSpeed(0.0),
      nextToLastSpeed(0.0),
      lastAcceleration(0.0)
{
    lastStamp.update();
}

// -----------------------------------------------------------------------------

void EncoderRead::update(std::int32_t newPos, double newTime)
{
    std::lock_guard<std::mutex> guard(encoderMutex);

    const double lastTime = lastStamp.getTime();

    nextToLastPosition = lastPosition;
    nextToLastSpeed = lastSpeed;

    if (newTime != 0.0)
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

void EncoderRead::reset(std::int32_t pos)
{
    std::lock_guard<std::mutex> guard(encoderMutex);
    lastPosition = nextToLastPosition = pos;
    lastSpeed = nextToLastSpeed = lastAcceleration = 0.0;
    lastStamp.update();
}

// -----------------------------------------------------------------------------

std::int32_t EncoderRead::queryPosition() const
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

bool StateVariables::validateInitialState()
{
    if (actualControlMode == 0)
    {
        CD_WARNING("Illegal initial control mode.\n");
        return false;
    }

    if (tr <= 0.0)
    {
        CD_WARNING("Illegal transmission ratio: %f.\n", tr.load());
        return false;
    }

    if (k <= 0.0)
    {
        CD_WARNING("Illegal motor constant: %f.\n", k.load());
        return false;
    }

    if (encoderPulses <= 0)
    {
        CD_WARNING("Illegal encoder pulses per revolution: %d.\n", encoderPulses.load());
        return false;
    }

    if (maxVel <= 0.0)
    {
        CD_WARNING("Illegal maximum velocity: %f.\n", maxVel.load());
        return false;
    }

    if (refSpeed <= 0.0)
    {
        CD_WARNING("Illegal reference speed: %f.\n", refSpeed.load());
        return false;
    }

    if (refAcceleration <= 0.0)
    {
        CD_WARNING("Illegal reference acceleration: %f.\n", refAcceleration.load());
        return false;
    }

    if (drivePeakCurrent <= 0.0)
    {
        CD_WARNING("Illegal drive peak current: %f.\n", drivePeakCurrent);
        return false;
    }

    if (pulsesPerSample <= 0)
    {
        CD_WARNING("Illegal pulses per sample: %d.\n", pulsesPerSample);
        return false;
    }

    if (refSpeed > maxVel)
    {
        CD_WARNING("Reference speed greater than maximum velocity: %f > %f.\n", refSpeed.load(), maxVel.load());
        return false;
    }

    if (min >= max)
    {
        CD_WARNING("Illegal joint limits (min, max): %f >= %f.\n", min.load(), max.load());
        return false;
    }

    if (axisName.empty())
    {
        CD_WARNING("Empty string as axis name.\n");
        return false;
    }

    switch (jointType)
    {
    case yarp::dev::VOCAB_JOINTTYPE_REVOLUTE:
    case yarp::dev::VOCAB_JOINTTYPE_PRISMATIC:
    case yarp::dev::VOCAB_JOINTTYPE_UNKNOWN:
        break;
    default:
        CD_WARNING("Illegal joint type vocab: %s.\n", yarp::os::Vocab::decode(jointType).c_str());
        return false;
    }

    if (heartbeatPeriod <= 0.0)
    {
        CD_WARNING("Illegal heartbeat period: %f.\n", heartbeatPeriod);
        return false;
    }

    if (heartbeatPeriod * 1e3 != static_cast<int>(heartbeatPeriod * 1e3))
    {
        CD_WARNING("Heartbeat period exceeds millisecond precision: %f (s).\n", heartbeatPeriod);
        return false;
    }

    if (syncPeriod <= 0.0 || syncPeriod > 255.0)
    {
        CD_WARNING("Illegal SYNC period: %f.\n", syncPeriod);
        return false;
    }

    if (syncPeriod * 1e3 != static_cast<int>(syncPeriod * 1e3))
    {
        CD_WARNING("SYNC period exceeds millisecond precision: %f (s).\n", syncPeriod);
        return false;
    }

    if (canId == 0)
    {
        CD_WARNING("Illegal CAN ID: %d.\n", canId);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

double StateVariables::clipSyncPositionTarget()
{
    double requested = synchronousCommandTarget;
    double previous = prevSyncTarget;
    double diff = requested - previous;

    if (std::abs(diff) > maxVel * syncPeriod)
    {
        CD_WARNING("Maximum velocity exceeded, clipping target position (canId: %d).\n", canId);
        double newTarget = previous + maxVel * syncPeriod * sgn(diff);
        prevSyncTarget = newTarget;
        return newTarget;
    }
    else
    {
        return requested;
    }
}

// -----------------------------------------------------------------------------

void StateVariables::reset()
{
    msr = mer = der = der2 = cer = ptStatus = 0;
    modesOfOperation = 0;

    lastEncoderRead.reset();
    lastCurrentRead = 0.0;

    requestedcontrolMode = 0;
    synchronousCommandTarget = prevSyncTarget = 0.0;
}

// -----------------------------------------------------------------------------

bool StateVariables::awaitControlMode(yarp::conf::vocab32_t mode)
{
    return actualControlMode == mode || controlModeObserverPtr->await();
}

// -----------------------------------------------------------------------------

double StateVariables::degreesToInternalUnits(double value, int derivativeOrder) const
{
    return value * tr * (reverse ? -1 : 1) * (encoderPulses / 360.0) * std::pow(1.0 / pulsesPerSample, derivativeOrder);
}

// -----------------------------------------------------------------------------

double StateVariables::internalUnitsToDegrees(double value, int derivativeOrder) const
{
    return value / (tr * (reverse ? -1 : 1) * (encoderPulses / 360.0) * std::pow(1.0 / pulsesPerSample, derivativeOrder));
}

// -----------------------------------------------------------------------------

std::int16_t StateVariables::currentToInternalUnits(double value) const
{
    return value * (reverse ? -1 : 1) * 65520.0 / (2.0 * drivePeakCurrent);
}

// -----------------------------------------------------------------------------

double StateVariables::internalUnitsToCurrent(std::int16_t value) const
{
    return value * (reverse ? -1 : 1) * 2.0 * drivePeakCurrent / 65520.0;
}

// -----------------------------------------------------------------------------

double StateVariables::internalUnitsToPeakCurrent(std::int16_t value) const
{
    return 2.0 * drivePeakCurrent * (32767.0 - value) / 65520.0;
}

// -----------------------------------------------------------------------------

double StateVariables::currentToTorque(double current) const
{
    return current * tr * k;
}

// -----------------------------------------------------------------------------

double StateVariables::torqueToCurrent(double torque) const
{
    return torque / (tr * k);
}

// -----------------------------------------------------------------------------
