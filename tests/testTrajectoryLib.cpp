#include "gtest/gtest.h"

#include <cmath> // std::abs

#include <utility> // std::swap

#include "TrapezoidalTrajectory.hpp"

#define EPSILON 1e-6

#define REF_ASSERT(ref, pos, vel, acc) do { \
        const auto & reference = ref; \
        ASSERT_NEAR(reference.position, pos, EPSILON); \
        ASSERT_NEAR(reference.velocity, vel, EPSILON); \
        ASSERT_NEAR(reference.acceleration, acc, EPSILON); \
    } while (0)

namespace roboticslab::test
{

/**
 * @ingroup yarp_devices_tests
 * @defgroup testTrajectoryLib
 * @brief Unit tests related to @ref TrajectoryLib.
 */

/**
 * @ingroup testTrajectoryLib
 * @brief Manages an instance of TrapezoidalTrajectory.
 */
class TrajectoryTest : public testing::Test
{
public:
    void SetUp() override
    {}

    void TearDown() override
    {}

protected:
    TrapezoidalTrajectory trajectory;

    static constexpr double ts = 123.456; // some random number representing a timestamp
};

TEST_F(TrajectoryTest, Inactive)
{
    ASSERT_FALSE(trajectory.isActive());
    ASSERT_NEAR(trajectory.getTargetPosition(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryPosition(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryVelocity(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryAcceleration(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryTime(), 0.0, EPSILON);

    trajectory.update(ts);

    ASSERT_FALSE(trajectory.isActive());
    ASSERT_NEAR(trajectory.getTargetPosition(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryPosition(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryVelocity(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryAcceleration(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryTime(), 0.0, EPSILON);
}

TEST_F(TrajectoryTest, Reset)
{
    double initialPosition = 10.0;

    trajectory.reset(initialPosition);

    ASSERT_FALSE(trajectory.isActive());
    ASSERT_NEAR(trajectory.getTargetPosition(), initialPosition, EPSILON);
    ASSERT_NEAR(trajectory.queryPosition(), initialPosition, EPSILON);
    ASSERT_NEAR(trajectory.queryVelocity(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryAcceleration(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryTime(), 0.0, EPSILON);

    trajectory.update(ts);

    ASSERT_FALSE(trajectory.isActive());
    ASSERT_NEAR(trajectory.getTargetPosition(), initialPosition, EPSILON);
    ASSERT_NEAR(trajectory.queryPosition(), initialPosition, EPSILON);
    ASSERT_NEAR(trajectory.queryVelocity(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryAcceleration(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryTime(), 0.0, EPSILON);
}

TEST_F(TrajectoryTest, NoDisplacement)
{
    double initialPosition = 10.0;
    double initialVelocity = 0.0;
    double refSpeed = 4.0;
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, initialPosition, refSpeed, refAcceleration);
    ASSERT_NEAR(trajectory.getTargetPosition(), initialPosition, EPSILON);

    ASSERT_FALSE(trajectory.isActive());
    ASSERT_NEAR(trajectory.queryPosition(), initialPosition, EPSILON);
    ASSERT_NEAR(trajectory.queryVelocity(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryAcceleration(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryTime(), 0.0, EPSILON);

    trajectory.update(ts);

    ASSERT_FALSE(trajectory.isActive());
    ASSERT_NEAR(trajectory.queryPosition(), initialPosition, EPSILON);
    ASSERT_NEAR(trajectory.queryVelocity(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryAcceleration(), 0.0, EPSILON);
    ASSERT_NEAR(trajectory.queryTime(), 0.0, EPSILON);
}

TEST_F(TrajectoryTest, FullTrapezoidal)
{
    double initialPosition = 10.0;
    double targetPosition = 40.0;
    double refSpeed = 6.0;
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, 0.0, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, 0.0, refAcceleration); // start of acceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), 19.0, refSpeed, 0.0); // start of constant speed interval
    REF_ASSERT(trajectory.update(ts + 5.0), 31.0, refSpeed, -refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 6.0), 36.0, 4.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 8.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    std::swap(initialPosition, targetPosition); // reverse direction

    trajectory.setTargetPosition(ts, initialPosition, 0.0, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, 0.0, -refAcceleration); // start of acceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), 31.0, -refSpeed, 0.0); // start of constant speed interval
    REF_ASSERT(trajectory.update(ts + 5.0), 19.0, -refSpeed, refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 6.0), 14.0, -4.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 8.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

TEST_F(TrajectoryTest, FullTriangular)
{
    double initialPosition = 10.0;
    double targetPosition = 18.0;
    double refSpeed = 6.0;
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, 0.0, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, 0.0, refAcceleration); // start of acceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), 14.0, 4.0, -refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), 17.0, 2.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    std::swap(initialPosition, targetPosition); // reverse direction

    trajectory.setTargetPosition(ts, initialPosition, 0.0, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, 0.0, -refAcceleration); // start of acceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), 14.0, -4.0, refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), 11.0, -2.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

TEST_F(TrajectoryTest, PositiveOffsetTrapezoidal)
{
    double initialPosition = 11.0;
    double targetPosition = 40.0;
    double initialVelocity = 2.0;
    double refSpeed = 6.0;
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, refAcceleration); // start of acceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), 19.0, refSpeed, 0.0); // start of constant speed interval
    REF_ASSERT(trajectory.update(ts + 4.0), 31.0, refSpeed, -refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 5.0), 36.0, 4.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 7.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    // reverse direction
    std::swap(initialPosition, targetPosition);
    initialVelocity = -initialVelocity;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, -refAcceleration); // start of acceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), 32.0, -refSpeed, 0.0); // start of constant speed interval
    REF_ASSERT(trajectory.update(ts + 4.0), 20.0, -refSpeed, refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 5.0), 15.0, -4.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 7.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

TEST_F(TrajectoryTest, PositiveOffsetTriangular)
{
    double initialPosition = 11.0;
    double targetPosition = 28.0;
    double initialVelocity = 2.0;
    double refSpeed = 10.0;
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, refAcceleration); // start of acceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), 19.0, 6.0, -refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), 24.0, 4.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 5.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    // reverse direction
    std::swap(initialPosition, targetPosition);
    initialVelocity = -initialVelocity;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, -refAcceleration); // start of acceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), 20.0, -6.0, refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), 15.0, -4.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 5.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

TEST_F(TrajectoryTest, NegativeOffsetTrapezoidal)
{
    double initialPosition = 11.0;
    double targetPosition = 30.0;
    double initialVelocity = -2.0; // initial backtrack
    double refSpeed = 4.0;
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, refAcceleration); // start of backtrack, deceleration ramp
    REF_ASSERT(trajectory.update(ts + 1.0), 10.0, 0.0, refAcceleration); // end of backtrack, start of acceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), 14.0, refSpeed, 0.0); // start of constant speed interval
    REF_ASSERT(trajectory.update(ts + 6.0), 26.0, refSpeed, -refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 7.0), 29.0, 2.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 8.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    // reverse direction
    std::swap(initialPosition, targetPosition);
    initialVelocity = -initialVelocity;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, -refAcceleration); // start of backtrack, deceleration ramp
    REF_ASSERT(trajectory.update(ts + 1.0), 31.0, 0.0, -refAcceleration); // end of backtrack, start of acceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), 27.0, -refSpeed, 0.0); // start of constant speed interval
    REF_ASSERT(trajectory.update(ts + 6.0), 15.0, -refSpeed, refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 7.0), 12.0, -2.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 8.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

TEST_F(TrajectoryTest, NegativeOffsetTriangular)
{
    double initialPosition = 11.0;
    double targetPosition = 18.0;
    double initialVelocity = -2.0; // initial backtrack
    double refSpeed = 6.0;
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, refAcceleration); // start of backtrack, deceleration ramp
    REF_ASSERT(trajectory.update(ts + 1.0), 10.0, 0.0, refAcceleration); // end of backtrack, start of acceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), 14.0, 4.0, -refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), 17.0, 2.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 5.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    // reverse direction
    std::swap(initialPosition, targetPosition);
    initialVelocity = -initialVelocity;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, -refAcceleration); // start of backtrack, deceleration ramp
    REF_ASSERT(trajectory.update(ts + 1.0), 19.0, 0.0, -refAcceleration); // end of backtrack, start of acceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), 15.0, -4.0, refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), 12.0, -2.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 5.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

TEST_F(TrajectoryTest, OvershootTrapezoidal)
{
    double initialPosition = 10.0;
    double targetPosition = 14.0;
    double initialVelocity = 8.0;
    double refSpeed = 4.0;
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), 26.0, 0.0, -refAcceleration); // start of backtrack and acceleration ramp
    REF_ASSERT(trajectory.update(ts + 6.0), 22.0, -refSpeed, 0.0); // start of constant speed interval
    REF_ASSERT(trajectory.update(ts + 7.0), 18.0, -refSpeed, refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 8.0), 15.0, -2.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 9.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    // reverse direction
    std::swap(initialPosition, targetPosition);
    initialVelocity = -initialVelocity;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), -2.0, 0.0, refAcceleration); // start of backtrack and acceleration ramp
    REF_ASSERT(trajectory.update(ts + 6.0), 2.0, refSpeed, 0.0); // start of constant speed interval
    REF_ASSERT(trajectory.update(ts + 7.0), 6.0, refSpeed, -refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 8.0), 9.0, 2.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 9.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

TEST_F(TrajectoryTest, OvershootTriangular)
{
    double initialPosition = 10.0;
    double targetPosition = 18.0;
    double initialVelocity = 8.0;
    double refSpeed = 6.0;
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), 26.0, 0.0, -refAcceleration); // start of backtrack and acceleration ramp
    REF_ASSERT(trajectory.update(ts + 6.0), 22.0, -4.0, refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 7.0), 19.0, -2.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 8.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    // reverse direction
    std::swap(initialPosition, targetPosition);
    initialVelocity = -initialVelocity;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), 2.0, 0.0, refAcceleration); // start of backtrack and acceleration ramp
    REF_ASSERT(trajectory.update(ts + 6.0), 6.0, 4.0, -refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 7.0), 9.0, 2.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 8.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

TEST_F(TrajectoryTest, OvershootNoDisplacement)
{
    double initialPosition = 10.0;
    double targetPosition = initialPosition;
    double initialVelocity = 8.0;
    double refSpeed = 4.0;
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), 26.0, 0.0, -refAcceleration); // start of backtrack and acceleration ramp
    REF_ASSERT(trajectory.update(ts + 6.0), 22.0, -refSpeed, 0.0); // start of constant speed interval
    REF_ASSERT(trajectory.update(ts + 8.0), 14.0, -refSpeed, refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 9.0), 11.0, -2.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 10.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    initialVelocity = -initialVelocity; // reverse direction

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), -6.0, 0.0, refAcceleration); // start of backtrack and acceleration ramp
    REF_ASSERT(trajectory.update(ts + 6.0), -2.0, refSpeed, 0.0); // start of constant speed interval
    REF_ASSERT(trajectory.update(ts + 8.0), 6.0, refSpeed, -refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 9.0), 9.0, 2.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 10.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    // in order to test a triangular profile, one would need to conveniently correlate `initialVelocity` and `refSpeed`
    // according to the formula `initialVelocity^2 = 2 * refSpeed^2`; however, the current implementation makes it
    // unfeasible to ensure that the correct branch is executed due to decimal precision (`deltaTotal` = `deltaTriMax`)
}

TEST_F(TrajectoryTest, Saddle)
{
    double initialPosition = 10.0;
    double targetPosition = 30.0;
    double initialVelocity = 8.0;
    double refSpeed = 4.0;
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, -refAcceleration); // first deceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), 22.0, refSpeed, 0.0); // start of constant speed interval
    REF_ASSERT(trajectory.update(ts + 3.0), 26.0, refSpeed, -refAcceleration); // start of second deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), 29.0, 2.0, -refAcceleration); // second deceleration ramp
    REF_ASSERT(trajectory.update(ts + 5.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    // reverse direction
    std::swap(initialPosition, targetPosition);
    initialVelocity = -initialVelocity;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, refAcceleration); // first deceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), 18.0, -refSpeed, 0.0); // start of constant speed interval
    REF_ASSERT(trajectory.update(ts + 3.0), 14.0, -refSpeed, refAcceleration); // start of second deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), 11.0, -2.0, refAcceleration); // second deceleration ramp
    REF_ASSERT(trajectory.update(ts + 5.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

TEST_F(TrajectoryTest, RightHalfTrapezoidal)
{
    double initialPosition = 10.0;
    double targetPosition = 22.0;
    double initialVelocity = 4.0;
    double refSpeed = std::abs(initialVelocity);
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, 0.0); // constant speed interval
    REF_ASSERT(trajectory.update(ts + 2.0), 18.0, refSpeed, -refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), 21.0, 2.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    // reverse direction
    std::swap(initialPosition, targetPosition);
    initialVelocity = -initialVelocity;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, 0.0); // constant speed interval
    REF_ASSERT(trajectory.update(ts + 2.0), 14.0, -refSpeed, refAcceleration); // start of deceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), 11.0, -2.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 4.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

TEST_F(TrajectoryTest, RightHalfTriangular)
{
    // corner case: testing `initialVelocity < refSpeed`

    double initialPosition = 10.0;
    double targetPosition = 14.0;
    double initialVelocity = 4.0;
    double refSpeed = 6.0;
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 1.0), 13.0, 2.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    // reverse direction
    std::swap(initialPosition, targetPosition);
    initialVelocity = -initialVelocity;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 1.0), 11.0, -2.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

TEST_F(TrajectoryTest, RightHalfTriangular2)
{
    // corner case: testing `initialVelocity > refSpeed`

    double initialPosition = 10.0;
    double targetPosition = 19.0;
    double initialVelocity = 6.0;
    double refSpeed = 4.0;
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 1.0), 15.0, 4.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    // reverse direction
    std::swap(initialPosition, targetPosition);
    initialVelocity = -initialVelocity;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 1.0), 14.0, -4.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

TEST_F(TrajectoryTest, RightHalfTriangular3)
{
    // corner case: testing `refSpeed == initialVelocity`

    double initialPosition = 10.0;
    double targetPosition = 14.0;
    double initialVelocity = 4.0;
    double refSpeed = std::abs(initialVelocity);
    double refAcceleration = 2.0;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 1.0), 13.0, 2.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    // reverse direction
    std::swap(initialPosition, targetPosition);
    initialVelocity = -initialVelocity;

    trajectory.setTargetPosition(ts, initialPosition, initialVelocity, targetPosition, refSpeed, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 1.0), 11.0, -2.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), targetPosition, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

TEST_F(TrajectoryTest, LeftHalfTrapezoidal)
{
    double initialPosition = 20.0;
    double initialVelocity = 4.0;
    double targetVelocity = 8.0;
    double refAcceleration = 2.0;

    trajectory.setTargetVelocity(ts, initialPosition, initialVelocity, targetVelocity, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, refAcceleration); // acceleration ramp
    REF_ASSERT(trajectory.update(ts + 1.0), 25.0, 6.0, refAcceleration); // acceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), 32.0, targetVelocity, 0.0); // start of infinite constant velocity interval

    ASSERT_TRUE(trajectory.isActive());

    targetVelocity = -targetVelocity; // reverse direction

    trajectory.setTargetVelocity(ts, initialPosition, initialVelocity, targetVelocity, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), 24.0, 0.0, -refAcceleration); // start of acceleration ramp
    REF_ASSERT(trajectory.update(ts + 3.0), 23.0, -2.0, -refAcceleration); // acceleration ramp
    REF_ASSERT(trajectory.update(ts + 6.0), 8.0, targetVelocity, 0.0); // start of infinite constant velocity interval

    ASSERT_TRUE(trajectory.isActive());
}

TEST_F(TrajectoryTest, LeftHalfRectangular)
{
    double initialPosition = 20.0;
    double initialVelocity = 4.0;
    double targetVelocity = initialVelocity;
    double refAcceleration = 2.0;

    trajectory.setTargetVelocity(ts, initialPosition, initialVelocity, targetVelocity, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, 0.0); // start of infinite constant velocity interval
    REF_ASSERT(trajectory.update(ts + 1.0), 24.0, targetVelocity, 0.0); // infinite constant velocity interval

    ASSERT_TRUE(trajectory.isActive());

    initialVelocity = targetVelocity = -initialVelocity; // reverse direction

    trajectory.setTargetVelocity(ts, initialPosition, initialVelocity, targetVelocity, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, 0.0); // start of infinite constant velocity interval
    REF_ASSERT(trajectory.update(ts + 1.0), 16.0, targetVelocity, 0.0); // infinite constant velocity interval

    ASSERT_TRUE(trajectory.isActive());
}

TEST_F(TrajectoryTest, Stop)
{
    double initialPosition = 20.0;
    double initialVelocity = 4.0;
    double targetVelocity = 0.0;
    double refAcceleration = 2.0;

    trajectory.setTargetVelocity(ts, initialPosition, initialVelocity, targetVelocity, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 1.0), 23.0, 2.0, -refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), 24.0, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());

    initialVelocity = -initialVelocity; // reverse direction

    trajectory.setTargetVelocity(ts, initialPosition, initialVelocity, targetVelocity, refAcceleration);

    ASSERT_TRUE(trajectory.isActive());

    REF_ASSERT(trajectory.update(ts), initialPosition, initialVelocity, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 1.0), 17.0, -2.0, refAcceleration); // deceleration ramp
    REF_ASSERT(trajectory.update(ts + 2.0), 16.0, 0.0, 0.0); // trajectory completed

    ASSERT_FALSE(trajectory.isActive());
}

} // namespace roboticslab::test
