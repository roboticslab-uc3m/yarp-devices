// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TrapezoidalTrajectory.hpp"

#include <cmath> // std::abs, std::copysign, std::sqrt

#include <chrono>
#include <limits>

using namespace roboticslab;

// -------------------------------------------------------------------------------------

namespace
{
    // https://stackoverflow.com/a/4609795/10404307
    template <typename T>
    inline int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    // from yarp/os/SystemClock.cpp
    inline double now()
    {
        return std::chrono::time_point_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();
    }
}

// -------------------------------------------------------------------------------------

TrapezoidalTrajectory::TrapezoidalTrajectory()
    : ta(0.0), a1(0.0), a2(0.0), a3(0.0),
      tb(0.0), b2(0.0), b3(0.0),
      tc(0.0), c1(0.0), c2(0.0), c3(0.0),
      startTimestamp(0.0),
      targetPosition(0.0),
      positionReference(0.0),
      velocityReference(0.0),
      active(false)
{}

// -------------------------------------------------------------------------------------

void TrapezoidalTrajectory::configure(const double initialPosition, const double targetPosition, const double speed, const double acceleration)
{
    if (speed <= 0.0 || acceleration <= 0.0 || std::abs(initialPosition - targetPosition) < 0.1 /* [deg] */)
    {
        return; // should be checked by callers
    }

    std::lock_guard lock(mutex);

    this->targetPosition = targetPosition;

    // displacement along the desired trajectory; can be negative (depending on the direction of motion) and lesser
    // than the total distance travelled (if overshoot occurs and we need to backtrack, or if initially we are moving
    // in the opposite direction)
    const double deltaTotal = targetPosition - initialPosition;

    // considering initial speed (if nonzero) and input deceleration, distance travelled before stopping
    const double steepestStopDistance = 0.5 * velocityReference * velocityReference / acceleration; // > 0

    // equal to `speed` and `acceleration`, but their sign needs to be adjusted depending on the resulting trajectory
    double refSpeed, firstAccelerationRamp, secondAccelerationRamp;

    // depending on whether and how we were moving prior to initiating the current trajectory, i.e. what was the last
    // value of `velocityReference`, the ramps will be determined differently; whether a constant-speed interval is
    // actually necessary or not will be determined later

    if (sgn(velocityReference) == sgn(deltaTotal) && steepestStopDistance > std::abs(deltaTotal))
    {
        // initially moving in the same direction and overshoot happens, i.e. considering the reference acceleration we
        // don't decelerate fast enough to stop before reaching the target position, therefore we need to backtrack
        // later by moving in the opposite direction a bit

        refSpeed = std::copysign(speed, -deltaTotal); // constant speed interval during backtrack
        firstAccelerationRamp = std::copysign(acceleration, -deltaTotal); // deceleration and initial backtrack
        secondAccelerationRamp = -firstAccelerationRamp; // complete backtrack until target position
    }
    else if (sgn(velocityReference) == sgn(deltaTotal) && std::abs(velocityReference) > speed)
    {
        // initially moving in the same direction and faster than the reference speed, therefore we need to apply an
        // initial deceleration ramp to comply with it and then continue deceleration until the target is reached

        refSpeed = std::copysign(speed, deltaTotal);
        firstAccelerationRamp = secondAccelerationRamp = std::copysign(acceleration, -deltaTotal);
    }
    else
    {
        // general case: apply an initial acceleration ramp until the reference speed is reached (in the direction of
        // motion), then decelerate

        refSpeed = std::copysign(speed, deltaTotal);
        firstAccelerationRamp = std::copysign(acceleration, deltaTotal);
        secondAccelerationRamp = -firstAccelerationRamp;
    }

    // displacement (not equal to distance!) along each ramp
    const double firstRampDelta = 0.5 * (refSpeed * refSpeed - velocityReference * velocityReference) / firstAccelerationRamp;
    const double secondRampDelta = -0.5 * refSpeed * refSpeed / secondAccelerationRamp;

    // hypothetic distance traveled using a double-ramp (i.e. triangular) trajectory that reaches the reference speed
    const double deltaTriMax = firstRampDelta + secondRampDelta;

    // the parameters here are interpreted as follows:
    // - `ta`: finish timestamp of the first ramp interval, beginning at zero
    // - `a1`, `a2`, `a3`: f(t) = 0.5 * a1 * t^2 + a2 * t + a3
    // - `tb`: finish timestamp of the constant speed interval, if any
    // - `b2`, `b3`: f(t) = `b2` * t + b3
    // - `tc`: finish timestamp of the second ramp interval
    // - `c1`, `c2`, `c3`: f(t) = 0.5 * c1 * t^2 + c2 * t + c3

    if (std::abs(deltaTotal) > std::abs(deltaTriMax))
    {
        // trapezoidal profile: a double-ramp (i.e. triangular) trajectory is not enough to reach the target position,
        // therefore we need to apply a constant speed interval in the middle

        ta = (refSpeed - velocityReference) / firstAccelerationRamp;
        a1 = firstAccelerationRamp;
        a2 = velocityReference;
        a3 = initialPosition;

        tb = ta + (deltaTotal - deltaTriMax) / refSpeed;
        b2 = refSpeed;
        b3 = a3 + 0.5 * (refSpeed * refSpeed - velocityReference * velocityReference) / firstAccelerationRamp;

        tc = tb - refSpeed / secondAccelerationRamp;
        c1 = secondAccelerationRamp;
        c2 = refSpeed;
        c3 = b3 - 0.5 * refSpeed * refSpeed / secondAccelerationRamp;
    }
    else
    {
        // triangular profile: determine the speed we need to reach at the end of the first ramp interval so that the
        // second ramp interval ends exactly at the target position

        double peakVelocity = std::sqrt(2 * firstAccelerationRamp * secondAccelerationRamp * deltaTotal / (secondAccelerationRamp - firstAccelerationRamp));

        ta = (peakVelocity - velocityReference) / firstAccelerationRamp;
        a1 = firstAccelerationRamp;
        a2 = velocityReference;
        a3 = initialPosition;

        // no constant speed interval, thus merge into the previous interval
        tb = ta;
        b2 = b3 = 0.0;

        tc = ta - peakVelocity / secondAccelerationRamp;
        c1 = secondAccelerationRamp;
        c2 = peakVelocity;
        c3 = a3 + 0.5 * (peakVelocity * peakVelocity - velocityReference * velocityReference) / firstAccelerationRamp;
    }

    startTimestamp = now();
    positionReference = initialPosition;
    active = true;
}

// -------------------------------------------------------------------------------------

void TrapezoidalTrajectory::configure(double initialPosition, double targetVelocity, double acceleration)
{
    double targetPosition = std::copysign(std::numeric_limits<double>::infinity(), targetVelocity);
    return configure(initialPosition, targetPosition, std::abs(targetVelocity), acceleration);
}

// -------------------------------------------------------------------------------------

void TrapezoidalTrajectory::reset(double currentPosition)
{
    std::lock_guard lock(mutex);

    startTimestamp = now();
    positionReference = targetPosition = currentPosition; // updated
    velocityReference = 0.0;

    ta = a1 = a2 = a3 = 0.0;
    tb = b2 = b3 = 0.0;
    tc = c1 = c2 = c3 = 0.0;

    active = false;
}

// -------------------------------------------------------------------------------------

void TrapezoidalTrajectory::update(double timestamp)
{
    std::lock_guard lock(mutex);

    if (!active)
    {
        return;
    }

    const double t = timestamp - startTimestamp;

    if (t <= ta) // first ramp
    {
        positionReference = 0.5 * a1 * t * t + a2 * t + a3;
        velocityReference = a1 * t + a2;
    }
    else if (t > ta && t <= tb) // constant speed interval between ramps (if any)
    {
        positionReference = b2 * t + b3;
        velocityReference = b2;
    }
    else if (t >= tb && t < tc) // second ramp
    {
        positionReference = 0.5 * c1 * t * t + c2 * t + c3;
        velocityReference = c1 * t + c2;
    }
    else // finished
    {
        positionReference = targetPosition;
        velocityReference = 0.0;
        active = false;
    }
}

// -------------------------------------------------------------------------------------

double TrapezoidalTrajectory::getTargetPosition() const
{
    std::lock_guard lock(mutex);
    return targetPosition;
}

// -------------------------------------------------------------------------------------

double TrapezoidalTrajectory::queryPosition() const
{
    std::lock_guard lock(mutex);
    return positionReference;
}

// -------------------------------------------------------------------------------------

double TrapezoidalTrajectory::queryVelocity() const
{
    std::lock_guard lock(mutex);
    return velocityReference;
}

// -------------------------------------------------------------------------------------

double TrapezoidalTrajectory::queryTime() const
{
    std::lock_guard lock(mutex);
    return now() - startTimestamp;
}

// -------------------------------------------------------------------------------------

bool TrapezoidalTrajectory::isActive() const
{
    std::lock_guard lock(mutex);
    return active;
}

// -------------------------------------------------------------------------------------
