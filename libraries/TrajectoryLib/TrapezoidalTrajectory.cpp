// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TrapezoidalTrajectory.hpp"

#include <cmath> // std::abs, std::copysign, std::sqrt

#include <limits>

using namespace roboticslab;

namespace
{
    constexpr double epsilon = 1e-6;
}

// -------------------------------------------------------------------------------------

namespace
{
    // https://stackoverflow.com/a/4609795/10404307
    template <typename T>
    inline int sgn(T val) {
        return (T(0) < val) - (val < T(0));
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
      accelerationReference(0.0),
      elapsedTime(0.0),
      active(false)
{}

// -------------------------------------------------------------------------------------

void TrapezoidalTrajectory::setTargetPosition(double timestamp, double initialPosition, double initialVelocity,
                                              double target, double speed, double acceleration)
{
    if (speed <= 0.0 || acceleration <= 0.0 || std::abs(target - initialPosition) < epsilon && std::abs(initialVelocity) < epsilon)
    {
        reset(initialPosition);
        return;
    }

    configure(timestamp, initialPosition, initialVelocity, target, speed, acceleration);
}

// -------------------------------------------------------------------------------------

void TrapezoidalTrajectory::setTargetVelocity(double startTimestamp, double initialPosition, double initialVelocity,
                                              double targetVelocity, double acceleration)
{
    if (acceleration <= 0.0)
    {
        reset(initialPosition);
        return;
    }

    if (std::abs(targetVelocity) < epsilon)
    {
        // special case: we want to stop as soon as possible
        double targetPosition = initialPosition + 0.5 * initialVelocity * initialVelocity / std::copysign(acceleration, initialVelocity);
        configure(startTimestamp, initialPosition, initialVelocity, targetPosition, 0.0, acceleration);
        return;
    }

    // otherwise, move indefinitely at `targetVelocity`
    double targetPosition = std::copysign(std::numeric_limits<double>::infinity(), targetVelocity);
    configure(startTimestamp, initialPosition, initialVelocity, targetPosition, std::abs(targetVelocity), acceleration);
}

// -------------------------------------------------------------------------------------

void TrapezoidalTrajectory::configure(const double timestamp, const double initialPosition, const double initialVelocity,
                                      const double target, const double speed, const double acceleration)
{
    // displacement along the desired trajectory; can be negative (depending on the direction of motion) and lesser
    // than the total distance travelled due to backtracking (if overshoot occurs or if initially we are moving in the
    // opposite direction)
    const double deltaTotal = target - initialPosition;

    // distance travelled before stopping given the initial speed (can be zero) and input deceleration
    const double steepestStopDistance = 0.5 * initialVelocity * initialVelocity / acceleration; // >= 0

    // equal to `speed` and `acceleration`, but their sign needs to be adjusted depending on the resulting trajectory
    double refSpeed, firstAccelerationRamp, secondAccelerationRamp;

    bool isOvershooting = false;

    // depending on how we were moving prior to starting the current trajectory (`initialVelocity`), the ramps will be
    // determined differently; whether a constant-speed interval is actually necessary or not will be determined later

    if (
        // not initially at rest
        std::abs(initialVelocity) > epsilon &&
        // initially moving towards the target or zero distance, i.e. `deltaTotal` might be zero (`sgn(0.0) == 0`)
        sgn(initialVelocity) * sgn(deltaTotal) >= 0 &&
        // overshoot: cannot decelerate fast enough to stop before reaching the target position; enables backtracking
        steepestStopDistance > std::abs(deltaTotal))
    {
        refSpeed = std::copysign(speed, -initialVelocity); // constant speed interval while backtracking
        firstAccelerationRamp = std::copysign(acceleration, -initialVelocity);
        secondAccelerationRamp = -firstAccelerationRamp;

        isOvershooting = true;
    }
    else if (std::abs(initialVelocity) > speed && sgn(initialVelocity) == sgn(deltaTotal))
    {
        // initially moving towards the target and faster than the reference speed, therefore we need to apply an
        // initial deceleration ramp to comply with it and then continue deceleration until the target is reached

        // NOTE: this branch MUST be evaluated after the previous one, i.e. overshoot handling takes precedence; also
        // `deltaTotal` will never be zero here (`sgn(0.0) == 0`, and we have `sgn(initialVelocity) != 0`) nor in the
        // following branch, which could have messed up the `std::copysign` calls

        refSpeed = std::copysign(speed, deltaTotal);
        firstAccelerationRamp = secondAccelerationRamp = std::copysign(acceleration, -deltaTotal);
    }
    else
    {
        // acceleration until the reference speed is reached (in the direction of motion), then deceleration

        refSpeed = std::copysign(speed, deltaTotal);
        firstAccelerationRamp = std::copysign(acceleration, deltaTotal);
        secondAccelerationRamp = -firstAccelerationRamp;
    }

    // displacement (not distance!) along each ramp, starting at `initialVelocity` until `refSpeed` is reached
    const double firstRampDelta = 0.5 * (refSpeed * refSpeed - initialVelocity * initialVelocity) / firstAccelerationRamp;
    const double secondRampDelta = -0.5 * refSpeed * refSpeed / secondAccelerationRamp;

    // hypothetic displacement using a double-ramp (i.e. triangular) trajectory that reaches the reference speed
    const double deltaTriMax = firstRampDelta + secondRampDelta;

    // the parameters here are interpreted as follows:
    // - `ta`: finish timestamp of the first ramp interval, beginning at t = 0s
    // - `a1`, `a2`, `a3`: f(t) = 0.5 * a1 * t^2 + a2 * t + a3
    // - `tb`: finish timestamp of the constant speed interval, if any
    // - `b2`, `b3`: f(t) = `b2` * (t - ta) + b3
    // - `tc`: finish timestamp of the second ramp interval
    // - `c1`, `c2`, `c3`: f(t) = 0.5 * c1 * (t - tb)^2 + c2 * (t - tb) + c3

    std::lock_guard lock(mutex);

    if (std::abs(deltaTotal) >= std::abs(deltaTriMax) ^ isOvershooting || firstAccelerationRamp == secondAccelerationRamp)
    {
        // trapezoidal profile: a double-ramp (i.e. triangular) trajectory is not enough to reach the target position,
        // therefore we need to apply a constant speed interval in the middle

        // NOTE: `firstAccelerationRamp == secondAccelerationRamp` is a protection against the potential division by
        // zero in the next branch, which may or may not be entered depending on decimal precision in other comparisons

        ta = (refSpeed - initialVelocity) / firstAccelerationRamp;
        a1 = firstAccelerationRamp;
        a2 = initialVelocity;
        a3 = initialPosition;

        tb = ta + (deltaTotal - deltaTriMax) / refSpeed;
        b2 = refSpeed;
        b3 = a3 + firstRampDelta;

        tc = tb - refSpeed / secondAccelerationRamp;
        c1 = secondAccelerationRamp;
        c2 = refSpeed;
        c3 = target - secondRampDelta;
    }
    else
    {
        // triangular profile: determine the speed we need to reach at the end of the first ramp interval so that the
        // second ramp interval ends exactly at the target position

        // NOTE: it is perfectly fine to have `ta = tb = 0.0` here as a result of computing a half-triangular profile,
        // which otherwise would have resulted in a division by zero due to accelerations having the same sign

        const double peakVelocity = std::copysign(
            std::sqrt(
                (2 * firstAccelerationRamp * secondAccelerationRamp * deltaTotal + secondAccelerationRamp * initialVelocity * initialVelocity) /
                (secondAccelerationRamp - firstAccelerationRamp)
            ),
            !isOvershooting ? deltaTotal : -deltaTotal
        );

        ta = (peakVelocity - initialVelocity) / firstAccelerationRamp;
        a1 = firstAccelerationRamp;
        a2 = initialVelocity;
        a3 = initialPosition;

        // no constant speed interval, thus merge into the previous interval
        tb = ta;
        b2 = b3 = 0.0;

        tc = ta - peakVelocity / secondAccelerationRamp;
        c1 = secondAccelerationRamp;
        c2 = peakVelocity;
        c3 = a3 + 0.5 * (peakVelocity * peakVelocity - initialVelocity * initialVelocity) / firstAccelerationRamp;
    }

    startTimestamp = timestamp;
    targetPosition = target;

    positionReference = initialPosition;
    velocityReference = initialVelocity;
    accelerationReference = firstAccelerationRamp;

    active = true;
}

// -------------------------------------------------------------------------------------

void TrapezoidalTrajectory::reset(double currentPosition)
{
    std::lock_guard lock(mutex);

    startTimestamp = elapsedTime = 0.0;
    positionReference = targetPosition = currentPosition; // updated
    velocityReference = 0.0;
    accelerationReference = 0.0;

    ta = a1 = a2 = a3 = 0.0;
    tb = b2 = b3 = 0.0;
    tc = c1 = c2 = c3 = 0.0;

    active = false;
}

// -------------------------------------------------------------------------------------

TrapezoidalTrajectory::Reference TrapezoidalTrajectory::update(double timestamp)
{
    std::lock_guard lock(mutex);

    if (!active)
    {
        return {};
    }

    elapsedTime = timestamp - startTimestamp;

    if (elapsedTime < ta) // first ramp
    {
        return {
            positionReference = 0.5 * a1 * elapsedTime * elapsedTime + a2 * elapsedTime + a3,
            velocityReference = a1 * elapsedTime + a2,
            accelerationReference = a1
        };
    }
    else if (elapsedTime >= ta && elapsedTime < tb) // constant speed interval between ramps (if any)
    {
        return {
            positionReference = b2 * (elapsedTime - ta) + b3,
            velocityReference = b2,
            accelerationReference = 0.0
        };
    }
    else if (elapsedTime >= tb && elapsedTime < tc) // second ramp
    {
        const double t = elapsedTime - tb;

        return {
            positionReference = 0.5 * c1 * t * t + c2 * t + c3,
            velocityReference = c1 * t + c2,
            accelerationReference = c1
        };
    }
    else // finished
    {
        active = false;
        elapsedTime = 0.0;

        return {
            positionReference = targetPosition,
            velocityReference = 0.0,
            accelerationReference = 0.0
        };
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

double TrapezoidalTrajectory::queryAcceleration() const
{
    std::lock_guard lock(mutex);
    return accelerationReference;
}

// -------------------------------------------------------------------------------------

double TrapezoidalTrajectory::queryTime() const
{
    std::lock_guard lock(mutex);
    return elapsedTime;
}

// -------------------------------------------------------------------------------------

bool TrapezoidalTrajectory::isActive() const
{
    std::lock_guard lock(mutex);
    return active;
}

// -------------------------------------------------------------------------------------
