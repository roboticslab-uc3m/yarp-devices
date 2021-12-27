// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup exampleOfflineTrajectorySync exampleOfflineTrajectorySync
 * @brief Perform an offline trajectory via position commands with a fixed period.
 * @see @ref exampleOfflineTrajectorySync.cpp
 */

/**
 * @example{lineno} exampleOfflineTrajectorySync.cpp
 *
 * A constant-velocity, single-joint trajectory is generated with configurable final target, motion
 * speed and period between consecutive points. The `yarp::dev::IRemoteVariables` interface is used
 * to configure and switch to interpolated position (pi) mode on the real robot. Although prepared
 * for remote execution, this application could be rewritten to connect to a local instance of
 * @ref CanBusControlboard. The techniques showcased here are best aimed at trajectories fully
 * available offline (e.g. loaded from file) that assume a constant period between points. To fully
 * take advantage of this fact, all points are sent to the robot in a single batch, hence the flow
 * of the program is slightly different from what you'd expect (no delay function within a for-loop).
 *
 * Usage (showing default option values):
@verbatim
 exampleOfflineTrajectorySync --remote /teo/leftArm --joint 5 --speed 2.0 --target -20.0 --period 50 --ip pt
@endverbatim
 *
 * @see @ref exampleOfflineTrajectoryAsync.cpp Use this if the period between points is variable.
 * @see @ref exampleOfflineTrajectorySync.py
 * @see [Tutorial: Trajectory Execution](https://robots.uc3m.es/teo-developer-manual/tutorial/trajectories.html) (external)
 * @note This application is not suitable for simulation.
 */

#include <cmath>

#include <iostream>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/SystemClock.h>

#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/dev/PolyDriver.h>

constexpr auto DEFAULT_REMOTE = "/teo/leftArm";
constexpr auto DEFAULT_JOINT = 5;
constexpr auto DEFAULT_SPEED = 2.0; // deg/s
constexpr auto DEFAULT_TARGET = (-20.0);
constexpr auto DEFAULT_PERIOD_MS = 50;
constexpr auto DEFAULT_IP_MODE = "pt";

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    auto remote = rf.check("remote", yarp::os::Value(DEFAULT_REMOTE), "remote port").asString();
    auto joint = rf.check("joint", yarp::os::Value(DEFAULT_JOINT), "joint id").asInt32();
    auto speed = rf.check("speed", yarp::os::Value(DEFAULT_SPEED), "trajectory speed (deg/s)").asFloat64();
    auto target = rf.check("target", yarp::os::Value(DEFAULT_TARGET), "target position (deg)").asFloat64();
    auto period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD_MS), "command period (ms)").asInt32() * 0.001;
    auto ip = rf.check("ip", yarp::os::Value(DEFAULT_IP_MODE), "interpolation submode [pt|pvt]").asString();

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

    if (ip != "pt" && ip != "pvt")
    {
        yError() << "Illegal ip mode:" << ip;
        return 1;
    }

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    yarp::os::Property options {{"device", yarp::os::Value("remote_controlboard")},
                                {"local", yarp::os::Value("/exampleOfflineTrajectorySync")},
                                {"remote", yarp::os::Value(remote)},
                                {"writeStrict", yarp::os::Value("on")}};

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        yError() << "Remote device not available";
        return 1;
    }

    yarp::dev::IControlMode * mode;
    yarp::dev::IEncoders * enc;
    yarp::dev::IPositionControl * pos;
    yarp::dev::IPositionDirect * posd;
    yarp::dev::IRemoteVariables * var;

    if (!dd.view(mode) || !dd.view(enc) || !dd.view(pos) || !dd.view(posd) || !dd.view(var))
    {
        yError() << "Unable to acquire robot interfaces";
        return 1;
    }

    yarp::os::Property p {{"enable", yarp::os::Value(true)},
                          {"mode", yarp::os::Value(ip)},
                          {"periodMs", yarp::os::Value(period * 1000.0)}};

    yarp::os::Value v;
    v.asList()->addString("linInterp");
    v.asList()->addList().fromString(p.toString());

    if (!var->setRemoteVariable("all", {v}))
    {
        yError() << "Unable to set interpolation mode";
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

    int count = 1;
    double newDistance = 0.0;

    do
    {
        newDistance = count * increment;
        auto position = initialPos + newDistance;
        yInfo("[%d] New target: %f", count++, position);
        posd->setPosition(joint, position);
    }
    while (std::abs(distance) > std::abs(newDistance));

    bool motionDone;

    do
    {
        std::cout << "." << std::flush;
        yarp::os::SystemClock::delaySystem(0.1);
    }
    while (pos->checkMotionDone(&motionDone) && !motionDone);

    std::cout << " end" << std::endl;

    return 0;
}
