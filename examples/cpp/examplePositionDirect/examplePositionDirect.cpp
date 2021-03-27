// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup examplePositionDirect examplePositionDirect
 * @brief This example connects to a remote controlboard device and sends position direct commands.
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
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/dev/PolyDriver.h>

#define DEFAULT_REMOTE "/teo/leftArm"
#define DEFAULT_JOINT 5
#define DEFAULT_SPEED 2.0 // deg/s
#define DEFAULT_POS_TARGET (-10.0)
#define DEFAULT_POSD_TARGET (-20.0)
#define DEFAULT_POSD_PERIOD_MS 50
#define DEFAULT_IP_MODE "pt"

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    auto remote = rf.check("remote", yarp::os::Value(DEFAULT_REMOTE), "remote port").asString();
    auto jointId = rf.check("id", yarp::os::Value(DEFAULT_JOINT), "joint id").asInt32();
    auto speed = rf.check("speed", yarp::os::Value(DEFAULT_SPEED), "trajectory speed (deg/s)").asFloat64();
    auto posTarget = rf.check("pos", yarp::os::Value(DEFAULT_POS_TARGET), "target position for pos mode (deg)").asFloat64();
    auto posdTarget = rf.check("posd", yarp::os::Value(DEFAULT_POSD_TARGET), "target position for posd mode (deg)").asFloat64();
    auto period = rf.check("period", yarp::os::Value(DEFAULT_POSD_PERIOD_MS), "posd command period (ms)").asInt32() * 0.001;
    auto isBatch = rf.check("batch", "stream interpolation data in batches");
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

    if (isBatch && !rf.check("ip"))
    {
        yError() << "Option --batch only available in ip mode";
        return 1;
    }

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    yarp::os::Property options {{"device", yarp::os::Value("remote_controlboard")},
                                {"local", yarp::os::Value("/examplePositionDirect")},
                                {"remote", yarp::os::Value(remote)}};

    if (isBatch)
    {
        options.put("writeStrict", "on");
    }

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

    if (!dd.view(mode) || !dd.view(enc) || !dd.view(pos) || !dd.view(posd) || (rf.check("ip") && !dd.view(var)))
    {
        yError() << "Problems acquiring robot interfaces";
        return 1;
    }

    int numJoints;
    enc->getAxes(&numJoints);

    if (jointId < 0 || jointId > numJoints - 1)
    {
        yError("Illegal joint ID: %d (numJoints: %d)", jointId, numJoints);
        return 1;
    }

    yInfo() << "-- testing POSITION MODE --";

    if (!mode->setControlMode(jointId, VOCAB_CM_POSITION))
    {
        yError() << "Problems setting position control: POSITION";
        return 1;
    }

    yInfo() << "Moving joint" << jointId << "to" << posTarget << "degrees...";

    if (!pos->positionMove(jointId, posTarget))
    {
        yError() << "positionMove() failed";
        return 1;
    }

    bool motionDone = false;

    do
    {
        std::cout << "." << std::flush;
        yarp::os::SystemClock::delaySystem(0.5);
    }
    while (pos->checkMotionDone(&motionDone) && !motionDone);

    std::cout << " end" << std::endl;
    std::cin.get();

    yInfo() << "-- testing POSITION DIRECT --";

    if (rf.check("ip"))
    {
        yarp::os::Property dict {{"enable", yarp::os::Value(true)},
                                 {"mode", yarp::os::Value(ipMode)}};

        if (isBatch)
        {
            dict.put("periodMs", yarp::os::Value(period * 1000.0));
        }

        yarp::os::Value v;
        v.asList()->addString("linInterp");
        v.asList()->addList().fromString(dict.toString());

        if (!var->setRemoteVariable("all", {v}))
        {
            yError() << "Unable to set interpolation mode";
            return 1;
        }
    }

    if (!mode->setControlMode(jointId, VOCAB_CM_POSITION_DIRECT))
    {
        yError() << "Problems setting position control: POSITION_DIRECT";
        return 1;
    }

    double initialPos;

    if (!enc->getEncoder(jointId, &initialPos))
    {
        yError() << "getEncoders() failed";
        return 1;
    }

    yInfo() << "Current ENC value:" << initialPos;

    std::cin.get();

    yInfo() << "Moving joint" << jointId << "to" << posdTarget << "degrees...";

    const double distance = posdTarget - initialPos;
    const double increment = std::copysign(speed * period, distance);

    if (isBatch)
    {
        int count = 1;
        double newDistance = 0.0;

        do
        {
            newDistance = count * increment;
            auto position = initialPos + newDistance;
            yInfo("[%d] New target: %f", count++, position);
            posd->setPosition(jointId, position);
        }
        while (std::abs(distance) - std::abs(newDistance) > 1e-6);

        do
        {
            std::cout << "." << std::flush;
            yarp::os::SystemClock::delaySystem(0.5);
        }
        while (pos->checkMotionDone(&motionDone) && !motionDone);

        std::cout << " end" << std::endl;
    }
    else
    {
        yarp::os::Timer::TimerCallback callback = [=](const auto & event)
            {
                auto newDistance = event.runCount * increment;
                auto position = initialPos + newDistance;
                yInfo("[%d] New target: %f", event.runCount, position);
                posd->setPosition(jointId, position);
                return std::abs(distance) - std::abs(newDistance) > 1e-6;
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
    }

    return 0;
}
