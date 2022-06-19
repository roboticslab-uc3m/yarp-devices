// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StateVariables.hpp"

#include <cmath>

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>
#include <yarp/dev/IAxisInfo.h>

#include "LogComponent.hpp"

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
    std::lock_guard<std::mutex> guard(encoderMutex);

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
    std::lock_guard<std::mutex> guard(encoderMutex);
    lastPosition = pos;
    lastSpeed = lastAcceleration = 0.0;
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
        yCWarning(IPOS) << "Illegal initial control mode";
        return false;
    }

    if (tr <= 0.0)
    {
        yCWarning(IPOS) << "Illegal transmission ratio:" << tr.load();
        return false;
    }

    if (k <= 0.0)
    {
        yCWarning(IPOS) << "Illegal motor constant:" << k.load();
        return false;
    }

    if (encoderPulses <= 0)
    {
        yCWarning(IPOS) << "Illegal encoder pulses per revolution:" << encoderPulses.load();
        return false;
    }

    if (maxVel <= 0.0)
    {
        yCWarning(IPOS) << "Illegal maximum velocity:" << maxVel.load();
        return false;
    }

    if (refSpeed <= 0.0)
    {
        yCWarning(IPOS) << "Illegal reference speed:" << refSpeed.load();
        return false;
    }

    if (refAcceleration <= 0.0)
    {
        yCWarning(IPOS) << "Illegal reference acceleration:" << refAcceleration.load();
        return false;
    }

    if (drivePeakCurrent <= 0.0)
    {
        yCWarning(IPOS) << "Illegal drive peak current:" << drivePeakCurrent;
        return false;
    }

    if (samplingPeriod <= 0.0)
    {
        yCWarning(IPOS) << "Illegal sampling period:" << samplingPeriod;
        return false;
    }

    if (refSpeed > maxVel)
    {
        yCWarning(IPOS) << "Reference speed greater than maximum velocity:" << refSpeed.load() << ">" << maxVel.load();
        return false;
    }

    if (min >= max)
    {
        yCWarning(IPOS) << "Illegal joint limits (min, max):" << min.load() << ">=" << max.load();
        return false;
    }

    if (axisName.empty())
    {
        yCWarning(IPOS) << "Empty string as axis name";
        return false;
    }

    switch (jointType)
    {
    case yarp::dev::VOCAB_JOINTTYPE_REVOLUTE:
    case yarp::dev::VOCAB_JOINTTYPE_PRISMATIC:
    case yarp::dev::VOCAB_JOINTTYPE_UNKNOWN:
        break;
    default:
        yCWarning(IPOS) << "Illegal joint type vocab:" << yarp::os::Vocab32::decode(jointType);
        return false;
    }

    if (heartbeatPeriod < 0.0)
    {
        yCWarning(IPOS) << "Illegal heartbeat period:" << heartbeatPeriod;
        return false;
    }

    if (heartbeatPeriod * 1e3 != static_cast<int>(heartbeatPeriod * 1e3))
    {
        yCWarning(IPOS) << "Heartbeat period exceeds millisecond precision:" << heartbeatPeriod << "(s)";
        return false;
    }

    if (syncPeriod <= 0.0 || syncPeriod > 255.0)
    {
        yCWarning(IPOS) << "Illegal SYNC period:" << syncPeriod;
        return false;
    }

    if (syncPeriod * 1e3 != static_cast<int>(syncPeriod * 1e3))
    {
        yCWarning(IPOS) << "SYNC period exceeds millisecond precision:" << syncPeriod << "(s)";
        return false;
    }

    if (canId == 0)
    {
        yCWarning(IPOS) << "Illegal CAN ID:" << canId;
        return false;
    }

    lastEncoderRead = std::make_unique<EncoderRead>(samplingPeriod);

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
        yCIWarning(IPOS, "ID" + std::to_string(canId), "Maximum velocity exceeded, clipping target position");
        double newTarget = previous + maxVel * syncPeriod * sgn(diff);
        prevSyncTarget = newTarget;
        return newTarget;
    }
    else
    {
        prevSyncTarget.store(synchronousCommandTarget);
        return requested;
    }
}

// -----------------------------------------------------------------------------

void StateVariables::reset()
{
    msr = mer = der = der2 = cer = ipStatus = 0;
    modesOfOperation = 0;

    lastEncoderRead->reset();
    lastCurrentRead = 0.0;

    requestedcontrolMode = 0;
    synchronousCommandTarget = prevSyncTarget = 0.0;
    enableSync = false;

    ipBufferFilled = ipMotionStarted = ipBufferEnabled = false;
}

// -----------------------------------------------------------------------------

bool StateVariables::awaitControlMode(yarp::conf::vocab32_t mode)
{
    return actualControlMode == mode || controlModeObserverPtr->await();
}

// -----------------------------------------------------------------------------

double StateVariables::degreesToInternalUnits(double value, int derivativeOrder) const
{
    return value * tr * (reverse ? -1 : 1) * (encoderPulses / 360.0) * std::pow(samplingPeriod, derivativeOrder);
}

// -----------------------------------------------------------------------------

double StateVariables::internalUnitsToDegrees(double value, int derivativeOrder) const
{
    return value / (tr * (reverse ? -1 : 1) * (encoderPulses / 360.0) * std::pow(samplingPeriod, derivativeOrder));
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
