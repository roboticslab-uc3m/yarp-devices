// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup exampleOnlineTrajectoryRemotePush exampleOnlineTrajectoryRemotePush
 * @brief Perform an online trajectory via position commands, pushing new setpoints at the sender's discretion.
 * @see @ref exampleOnlineTrajectoryRemotePush.cpp
 */

/**
 * @example{lineno} exampleOnlineTrajectoryRemotePush.cpp
 *
 * A constant-velocity, single-joint trajectory is generated with configurable final target, motion
 * speed and period between consecutive points. The period is assumed constant and must be sent at
 * precise intervals if commanding the real robot (maps to cyclic synchronous position mode, a.k.a.
 * CSP). Although prepared for remote execution, this application could be rewritten to connect to
 * a local instance of @ref CanBusControlboard; however, a better alternative exists (see notes).
 * The techniques showcased here are especially suited for online-generated trajectories, e.g.
 * joystick teleoperation of visual servoing.
 *
 * Usage (showing default option values):
@verbatim
 exampleOnlineTrajectoryRemotePush --remote /teo/leftArm --joint 5 --speed 2.0 --target -20.0 --period 50
@endverbatim
 *
 * @warning If commanding the real robot, it is paramount that the `--period` option (milliseconds)
 * matches `--syncPeriod` (seconds) in @ref CanBusControlboard.
 * @see @ref exampleOnlineTrajectoryLocalPull.cpp Command a local instance of the real robot controller via
 * callback.
 * @see @ref exampleOnlineTrajectoryRemotePull.cpp Command a remote robot via callback, be it real or simulated.
 * @see @ref exampleOnlineTrajectoryRemotePush.py
 * @see [Tutorial: Trajectory Execution](https://robots.uc3m.es/teo-developer-manual/tutorial/trajectories.html) (external)
 */

#include <cmath>

#include <functional>
#include <iostream>
#include <utility>

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/SystemClock.h>

#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>

constexpr auto DEFAULT_REMOTE = "/teo/leftArm";
constexpr auto DEFAULT_JOINT = 5;
constexpr auto DEFAULT_SPEED = 2.0; // deg/s
constexpr auto DEFAULT_TARGET = (-20.0);
constexpr auto DEFAULT_PERIOD_MS = 50;

class Worker : public yarp::os::PeriodicThread
{
public:
    Worker(double period, double initial, double increment, double distance, std::function<void(double)> cmd)
#if YARP_VERSION_MINOR >= 5
        : yarp::os::PeriodicThread(period, yarp::os::PeriodicThreadClock::Absolute),
#else
        : yarp::os::PeriodicThread(period),
#endif
          command(std::move(cmd)),
          initial(initial),
          current(initial),
          increment(increment),
          distance(distance)
    {}

protected:
    void run() override
    {
        current += increment;
        yInfo("[%d] New target: %f", getIterations() + 1, current);
        command(current);

        if (std::abs(current - initial) >= std::abs(distance))
        {
            askToStop();
        }
    }

private:
    std::function<void(double)> command;
    const double initial;
    double current;
    const double increment;
    const double distance;
};

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    auto remote = rf.check("remote", yarp::os::Value(DEFAULT_REMOTE), "remote port").asString();
    auto joint = rf.check("joint", yarp::os::Value(DEFAULT_JOINT), "joint id").asInt32();
    auto speed = rf.check("speed", yarp::os::Value(DEFAULT_SPEED), "trajectory speed (deg/s)").asFloat64();
    auto target = rf.check("target", yarp::os::Value(DEFAULT_TARGET), "target position (deg)").asFloat64();
    auto period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD_MS), "command period (ms)").asInt32() * 0.001;

    if (speed <= 0)
    {
        yError() << "Illegal speed (deg/s):" << speed;
        return 1;
    }

    if (period <= 0)
    {
        yError() << "Illegal period (s):" << period;
        return 1;
    }

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    yarp::os::Property options {{"device", yarp::os::Value("remote_controlboard")},
                                {"local", yarp::os::Value("/exampleOnlineTrajectoryRemotePush")},
                                {"remote", yarp::os::Value(remote)}};

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        yError() << "Remote device not available";
        return 1;
    }

    yarp::dev::IControlMode * mode;
    yarp::dev::IEncoders * enc;
    yarp::dev::IPositionDirect * posd;

    if (!dd.view(mode) || !dd.view(enc) || !dd.view(posd))
    {
        yError() << "Unable to acquire robot interfaces";
        return 1;
    }

    if (!mode->setControlMode(joint, VOCAB_CM_POSITION_DIRECT))
    {
        yError() << "Unable to set position direct mode";
        return 1;
    }

    double initialPos;
    int retries = 0;

    while (!enc->getEncoder(joint, &initialPos) && retries++ < 10)
    {
        yarp::os::SystemClock::delaySystem(0.05);
    }

    if (retries >= 10)
    {
        yError() << "getEncoders() failed";
        return 1;
    }

    yInfo() << "Current ENC value:" << initialPos;

    std::cin.get();

    yInfo() << "Moving joint" << joint << "to" << target << "degrees...";

    const double distance = target - initialPos;
    const double increment = std::copysign(speed * period, distance);

    Worker worker(period, initialPos, increment, distance, [=](auto pos) { posd->setPosition(joint, pos); });

    if (!worker.start())
    {
        yError() << "Unable to start trajectory thread";
        return 1;
    }

    while (worker.isRunning())
    {
        yarp::os::SystemClock::delaySystem(0.1);
    }

    return 0;
}
