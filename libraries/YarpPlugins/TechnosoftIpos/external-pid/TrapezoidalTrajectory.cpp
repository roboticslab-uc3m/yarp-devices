// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TrapezoidalTrajectory.hpp"

#include <cmath>

using namespace roboticslab;

// -------------------------------------------------------------------------------------

TrapezoidalTrajectory::TrapezoidalTrajectory()
    : period(0.0),
      targetPosition(0.0),
      prevRefSpeed(0.0),
      refSpeed(0.0),
      refAcceleration(0.0),
      ta(0.0),
      tb(0.0),
      tc(0.0),
      tick(0),
      positionReference(0.0),
      velocityReference(0.0),
      active(false)
{}

// -------------------------------------------------------------------------------------

void TrapezoidalTrajectory::configure(double period, double initial, double target, double refSpeed, double refAcceleration)
{
    std::lock_guard<std::mutex> lock(mutex);

    if (refSpeed < 0.0 || refAcceleration < 0.0)
    {
        return; // should be handled by caller
    }

    this->period = period;
    targetPosition = target;
    prevRefSpeed = velocityReference; // last computed velocity from previous trajectory

    double sign = (target - initial >= 0.0) ? 1.0 : -1.0;
    this->refSpeed = sign * refSpeed;
    this->refAcceleration = sign * refAcceleration;

    // hypothetic distance traveled using a double-ramp (i.e. triangular) trajectory that reaches the reference speed
    double deltaTriMax = 0.5 * (2 * refSpeed * refSpeed - prevRefSpeed * prevRefSpeed) / refAcceleration;
    double deltaTotal = target - initial;

    if (std::abs(deltaTotal) > std::abs(deltaTriMax))
    {
        // trapezoidal profile
        double deltaRamp1 = 0.5 * (refSpeed * refSpeed - prevRefSpeed * prevRefSpeed) / refAcceleration;
        double deltaRamp2 = 0.5 * refSpeed * refSpeed / refAcceleration;
        ta = (refSpeed - prevRefSpeed) / refAcceleration;
        tb = ta + (deltaTotal - deltaRamp1 - deltaRamp2) / refSpeed;
        tc = tb + (refSpeed / refAcceleration);
    }
    else
    {
        // triangular profile
        double peakVelocity = sign * std::sqrt(0.5 * (2 * refAcceleration * deltaTotal + prevRefSpeed * prevRefSpeed));
        ta = (peakVelocity - prevRefSpeed) / refAcceleration;
        tb = ta;
        tc = ta + (peakVelocity / refAcceleration);
    }

    tick = 0;
    positionReference = initial;
    active = true;
}

// -------------------------------------------------------------------------------------

void TrapezoidalTrajectory::reset(double currentPosition)
{
    std::lock_guard<std::mutex> lock(mutex);
    targetPosition = currentPosition; // updated
    prevRefSpeed = refSpeed = refAcceleration = 0.0;
    ta = tb = tc = 0.0;
    tick = 0;
    positionReference = currentPosition; // updated
    velocityReference = 0.0;
    active = false;
}

// -------------------------------------------------------------------------------------

void TrapezoidalTrajectory::update()
{
    std::lock_guard<std::mutex> lock(mutex);

    if (!active)
    {
        return;
    }

    tick++;

    const double t = tick * period; // current time since start (in seconds)
    double step = 0.0;

    if (t <= ta) // first ramp
    {
        step = refAcceleration * period * period * tick + prevRefSpeed * period;
        velocityReference = refAcceleration * t + prevRefSpeed;
    }
    else if (t > ta && t <= tb) // constant speed interval between ramps (if any)
    {
        step = refSpeed * period;
        velocityReference = refSpeed;
    }
    else if (t >= tb && t < tc) // second ramp
    {
        step = refAcceleration * (tc * period - period * period * tick);
        velocityReference = refAcceleration * (tc - t);

        if (std::abs(step) >= std::abs(targetPosition - positionReference))
        {
            step = targetPosition - positionReference;
        }
    }
    else // finished
    {
        step = velocityReference = 0.0;
        active = false;
    }

    positionReference += step;
}

// -------------------------------------------------------------------------------------

double TrapezoidalTrajectory::queryPosition() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return positionReference;
}

// -------------------------------------------------------------------------------------

double TrapezoidalTrajectory::queryVelocity() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return velocityReference;
}

// -------------------------------------------------------------------------------------

double TrapezoidalTrajectory::queryTime() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return tick * period;
}

// -------------------------------------------------------------------------------------

bool TrapezoidalTrajectory::isActive() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return active;
}

// -------------------------------------------------------------------------------------
