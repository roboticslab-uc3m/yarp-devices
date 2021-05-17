// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup exampleOnlineTrajectoryLocalPull exampleOnlineTrajectoryLocalPull
 * @brief Perform an online trajectory via position commands attending a local callback.
 * @see @ref exampleOnlineTrajectoryLocalPull.cpp
 */

/**
 * @example{lineno} exampleOnlineTrajectoryLocalPull.cpp
 *
 * A constant-velocity, single-joint trajectory is generated with configurable final target, motion
 * speed and period between consecutive points. The period is assumed constant (maps to cyclic
 * synchronous position mode on the real robot, a.k.a. CSP). A callback is registered for listening
 * to a local synchronization loop managed by @ref CanBusControlboard, position commands will be
 * prepared and sent in response. The techniques showcased here are especially suited for
 * online-generated trajectories, e.g. joystick teleoperation of visual servoing.
 *
 * Usage (showing default option values):
@verbatim
 exampleOnlineTrajectoryLocalPull --bus pcan-leftArm --ipos id26-ipos --speed 2.0 --target -20.0 --period 50
@endverbatim
 *
 * @see @ref exampleOnlineTrajectoryRemotePull.cpp Command a remote instance of the real or simulated robot
 * via callback on a YARP synchronization port.
 * @see @ref exampleOnlineTrajectoryRemotePush.cpp Command a remote robot via direct position commands.
 * @see [Tutorial: Trajectory Execution](https://roboticslab-uc3m.github.io/teo-developer-manual/tutorial/trajectories.html) (external)
 * @note This application is not suitable for simulation.
 */

#include <cmath>

#include <iostream>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/SystemClock.h>

#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>

#include <StateObserver.hpp>

#define DEFAULT_BUS "pcan-leftArm"
#define DEFAULT_IPOS "id26-ipos"
#define DEFAULT_SPEED 2.0 // deg/s
#define DEFAULT_TARGET (-20.0)
#define DEFAULT_PERIOD_MS 20

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    auto bus = rf.check("bus", yarp::os::Value(DEFAULT_BUS), "CAN bus group name").asString();
    auto ipos = rf.check("ipos", yarp::os::Value(DEFAULT_IPOS), "iPOS group name").asString();
    auto speed = rf.check("speed", yarp::os::Value(DEFAULT_SPEED), "trajectory speed (deg/s)").asFloat64();
    auto target = rf.check("target", yarp::os::Value(DEFAULT_TARGET), "target position (deg)").asFloat64();
    auto period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD_MS), "synchronization period (ms)").asInt32() * 0.001;

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

    yarp::os::Property robotConfig;
    auto * robotConfigPtr = &robotConfig;
    auto path = rf.findFileByName("config.ini");

    if (path.empty() || !robotConfig.fromConfigFile(path))
    {
        yError() << "Unable to load robot config file";
        return 1;
    }

    roboticslab::StateObserver syncObserver(1.0);
    auto * syncObserverPtr = &syncObserver;

    yarp::os::Property options;
    options.put("device", yarp::os::Value("CanBusControlboard"));
    options.put("buses", yarp::os::Value::makeList(bus.c_str()));
    options.put(bus, yarp::os::Value::makeList(ipos.c_str()));
    options.put("syncPeriod", yarp::os::Value(period));
    options.put("robotConfig", yarp::os::Value::makeBlob(&robotConfigPtr, sizeof(robotConfigPtr)));
    options.put("syncObserver", yarp::os::Value::makeBlob(&syncObserverPtr, sizeof(syncObserverPtr)));

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        yError() << "Local device not available";
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

    if (!mode->setControlMode(0, VOCAB_CM_POSITION_DIRECT))
    {
        yError() << "Unable to set position direct mode";
        return 1;
    }

    double initialPos;
    int retries = 0;

    while (!enc->getEncoder(0, &initialPos) && retries++ < 10)
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

    yInfo() << "Moving joint" << 0 << "to" << target << "degrees...";

    const double velocity = std::copysign(speed, target - initialPos);
    const double offset = yarp::os::SystemClock::nowSystem();

    double lastRef;

    while (posd->getRefPosition(0, &lastRef) && std::abs(lastRef - initialPos) < std::abs(target - initialPos))
    {
        if (!syncObserver.await())
        {
            yWarning() << "Sync observer timeout";
            break;
        }

        double t = yarp::os::SystemClock::nowSystem() - offset;
        double e = initialPos + velocity * t;
        posd->setPosition(0, e);
    }

    return 0;
}
