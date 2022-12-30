// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAPEZOIDAL_TRAJECTORY_HPP__
#define __TRAPEZOIDAL_TRAJECTORY_HPP__

#include <mutex>

namespace roboticslab
{

/**
 * @ingroup TechnosoftIpos
 * @brief Trapezoidal trajectory generator.
 */
class TrapezoidalTrajectory
{
public:
    //! Constructor.
    TrapezoidalTrajectory();

    //! Set motion parameters (using target position).
    void configure(double initialPosition, double targetPosition, double refSpeed, double refAcceleration);

    //! Set motion parameters (infinite motion).
    void configure(double initialPosition, double targetVelocity, double refAcceleration);

    //! Reset state, remember current position (units).
    void reset(double currentPosition);

    //! Update trajectory state, must be called on regular intervals of `period`.
    void update(double timestamp);

    //! Query configured target position (units).
    double getTargetPosition() const;

    //! Retrieve last position reference (units).
    double queryPosition() const;

    //! Retrieve last velocity reference (units/seconds).
    double queryVelocity() const;

    //! Retrieve last updated time since start (seconds).
    double queryTime() const;

    //! Whether the trajectory is currently ongoing.
    bool isActive() const;

private:
    double ta, a1, a2, a3;
    double tb, b2, b3;
    double tc, c1, c2, c3;

    double startTimestamp;
    double targetPosition;
    double positionReference;
    double velocityReference;

    bool active;
    mutable std::mutex mutex;
};

} // namespace roboticslab

#endif // __TRAPEZOIDAL_TRAJECTORY_HPP__
