// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAPEZOIDAL_TRAJECTORY_HPP__
#define __TRAPEZOIDAL_TRAJECTORY_HPP__

#include <mutex>

namespace roboticslab
{

/**
 * @ingroup yarp_devices_libraries
 * @defgroup TrajectoryLib
 * @brief Trapezoidal trajectory generator.
 */
class TrapezoidalTrajectory
{
public:
    struct Reference
    {
        double position; // units
        double velocity; // units/seconds
        double acceleration; // units/seconds^2
    };

    //! Constructor.
    TrapezoidalTrajectory();

    //! Set motion parameters (using target position).
    void setTargetPosition(double startTimestamp, double initialPosition, double initialVelocity, double targetPosition, double refSpeed, double refAcceleration);

    //! Set motion parameters (infinite motion).
    void setTargetVelocity(double startTimestamp, double initialPosition, double initialVelocity, double targetVelocity, double refAcceleration);

    //! Reset state, remember current position (units).
    void reset(double currentPosition);

    //! Update trajectory state, must be called on regular intervals of `period`.
    Reference update(double timestamp);

    //! Query configured target position (units).
    double getTargetPosition() const;

    //! Retrieve last position reference (units).
    double queryPosition() const;

    //! Retrieve last velocity reference (units/seconds).
    double queryVelocity() const;

    //! Retrieve last acceleration reference (units/seconds^2).
    double queryAcceleration() const;

    //! Retrieve elapsed time since start (seconds).
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
    double accelerationReference;
    double elapsedTime;

    bool active;
    mutable std::mutex mutex;
};

} // namespace roboticslab

#endif // __TRAPEZOIDAL_TRAJECTORY_HPP__
