// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup exampleOfflineTrajectoryAsync exampleOfflineTrajectoryAsync
 * @brief Perform an offline trajectory via position commands with a variable period.
 *
 * A constant-velocity, single-joint trajectory is generated with configurable final target, motion
 * speed and period between consecutive points. The `yarp::dev::IRemoteVariables` interface is used
 * to configure and switch to interpolated position (pi) mode on the real robot. Although prepared
 * for remote execution, this application could be rewritten to connect to a local instance of
 * @ref CanBusControlboard. The techniques showcased here are best aimed at trajectories fully
 * available offline (e.g. loaded from file) that don't assume a constant period between points.
 *
 * Usage (showing default option values):
@verbatim
 exampleOfflineTrajectoryAsync --remote /teo/leftArm --joint 5 --speed 2.0 --target -20.0 --period 50 --ip pt
@endverbatim
 *
 * @see exampleOfflineTrajectorySync Assumes the period between points is fixed, thus allowing batched
 * execution.
 * @note This application is not suitable for simulation.
 */

#include <cmath>

#include <iostream>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Timer.h>

#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/dev/PolyDriver.h>

#define DEFAULT_REMOTE "/teo/leftArm"
#define DEFAULT_JOINT 5
#define DEFAULT_SPEED 2.0 // deg/s
#define DEFAULT_TARGET (-20.0)
#define DEFAULT_PERIOD_MS 50
#define DEFAULT_IP_MODE "pt"

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    auto remote = rf.check("remote", yarp::os::Value(DEFAULT_REMOTE), "remote port").asString();
    auto joint = rf.check("joint", yarp::os::Value(DEFAULT_JOINT), "joint id").asInt32();
    auto speed = rf.check("speed", yarp::os::Value(DEFAULT_SPEED), "trajectory speed (deg/s)").asFloat64();
    auto target = rf.check("target", yarp::os::Value(DEFAULT_TARGET), "target position (deg)").asFloat64();
    auto period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD_MS), "command period (ms)").asInt32() * 0.001;
    auto ipMode = rf.check("ip", yarp::os::Value(DEFAULT_IP_MODE), "interpolation submode [pt|pvt]").asString();

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

    if (ipMode != "pt" && ipMode != "pvt")
    {
        yError() << "Illegal ipMode:" << ipMode;
        return 1;
    }

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    yarp::os::Property options {{"device", yarp::os::Value("remote_controlboard")},
                                {"local", yarp::os::Value("/exampleOfflineTrajectoryAsync")},
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
    yarp::dev::IRemoteVariables * var;

    if (!dd.view(mode) || !dd.view(enc) || !dd.view(posd) || !dd.view(var))
    {
        yError() << "Unable to acquire robot interfaces";
        return 1;
    }

    yarp::os::Property p {{"enable", yarp::os::Value(true)},
                          {"mode", yarp::os::Value(ipMode)}};

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

    if (!retries >= 10)
    {
        yError() << "getEncoders() failed";
        return 1;
    }

    yInfo() << "Current ENC value:" << initialPos;

    std::cin.get();

    yInfo() << "Moving joint" << joint << "to" << target << "degrees...";

    const double distance = target - initialPos;
    const double increment = std::copysign(speed * period, distance);

    yarp::os::Timer::TimerCallback callback = [=](const auto & event)
        {
            auto newDistance = event.runCount * increment;
            auto position = initialPos + newDistance;
            yInfo("[%d] New target: %f", event.runCount, position);
            posd->setPosition(joint, position);
            return std::abs(distance) > std::abs(newDistance);
        };

    yarp::os::TimerSettings settings(period); // alternative ctor supports stop conditions
    yarp::os::Timer timer(settings, callback, true);

    if (!timer.start())
    {
        yError() << "Unable to start trajectory thread";
        return 1;
    }

    while (timer.isRunning())
    {
        yarp::os::SystemClock::delaySystem(0.1);
    }

    return 0;
}