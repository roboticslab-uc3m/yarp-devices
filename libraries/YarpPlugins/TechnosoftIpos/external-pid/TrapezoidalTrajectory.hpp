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

    //! Set motion parameters.
    void configure(double period, double initial, double target, double refSpeed, double refAcceleration);

    //! Reset state, remember current position (units).
    void reset(double currentPosition);

    //! Update trajectory state, must be called on regular intervals of `period`.
    void update();

    //! Retrieve last position reference (units).
    double queryPosition() const;

    //! Retrieve last velocity reference (units/seconds).
    double queryVelocity() const;

    //! Retrieve last updated time since start (seconds).
    double queryTime() const;

    //! Whether the trajectory is currently ongoing.
    bool isActive() const;

private:
    double period;
    double targetPosition;
    double prevRefSpeed;
    double refSpeed;
    double refAcceleration;
    double ta;
    double tb;
    double tc;
    int tick;
    double positionReference;
    double velocityReference;
    bool active;
    mutable std::mutex mutex;
};

} // namespace roboticslab

#endif // __TRAPEZOIDAL_TRAJECTORY_HPP__
