// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup exampleOnlineTrajectoryRemotePull exampleOnlineTrajectoryRemotePull
 * @brief This example connects to a remote controlboard device and sends position direct commands.
 */

#include <cmath>

#include <functional>
#include <iostream>
#include <utility>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/TypedReaderCallback.h>

#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>

#define DEFAULT_ROBOT "/teo"
#define DEFAULT_PART "/leftArm"
#define DEFAULT_JOINT 5
#define DEFAULT_SPEED 2.0 // deg/s
#define DEFAULT_TARGET (-20.0)

class SyncCallback : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    SyncCallback(double initialPos, double speed, std::function<void(double)> cmd)
        : count(0), e0(initialPos), v(speed), offset(0.0), command(std::move(cmd))
    {}

    void onRead(yarp::os::Bottle & b) override
    {
        if (b.size() == 2)
        {
            double time = b.get(0).asInt32() + b.get(1).asInt32() * 1e-9;

            if (count == 0)
            {
                offset = time;
            }

            double t = time - offset;
            double e = e0 + v * t;

            yInfo("[%d] New target: %f", ++count, e);
            command(e);
        }
    }

private:
    int count;
    const double e0;
    const double v;
    double offset;
    std::function<void(double)> command;
};

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    auto robot = rf.check("robot", yarp::os::Value(DEFAULT_ROBOT), "robot port").asString();
    auto part = rf.check("part", yarp::os::Value(DEFAULT_PART), "part port").asString();
    auto jointId = rf.check("id", yarp::os::Value(DEFAULT_JOINT), "joint id").asInt32();
    auto speed = rf.check("speed", yarp::os::Value(DEFAULT_SPEED), "trajectory speed (deg/s)").asFloat64();
    auto target = rf.check("target", yarp::os::Value(DEFAULT_TARGET), "target position (deg)").asFloat64();

    if (speed <= 0)
    {
        yError() << "Illegal speed (deg/s):" << speed;
        return 1;
    }

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    yarp::os::Property options {{"device", yarp::os::Value("remote_controlboard")},
                                {"local", yarp::os::Value("/exampleOnlineTrajectoryRemotePull")},
                                {"remote", yarp::os::Value(robot + part)}};

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
        yError() << "Problems acquiring robot interfaces";
        return 1;
    }

    yarp::os::BufferedPort<yarp::os::Bottle> syncPort;
    syncPort.setReadOnly();

    if (!syncPort.open("/examplePositionDirectPullRemote/sync:i"))
    {
        yError() << "Unable to open local sync port";
        return 1;
    }

    if (!yarp::os::Network::connect(robot + "/sync:o", syncPort.getName(), "fast_tcp"))
    {
        yError() << "Unable to connect to remote sync port";
        return 1;
    }

    if (!mode->setControlMode(jointId, VOCAB_CM_POSITION_DIRECT))
    {
        yError() << "Unable to set position direct mode";
        return 1;
    }

    double initialPos;
    int retries = 0;

    while (!enc->getEncoder(jointId, &initialPos) && retries++ < 10)
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

    yInfo() << "Moving joint" << jointId << "to" << target << "degrees...";

    double velocity = std::copysign(speed, target - initialPos);

    SyncCallback callback(initialPos, velocity, [=](auto pos) { posd->setPosition(jointId, pos); });
    syncPort.useCallback(callback);

    double lastRef;

    while (posd->getRefPosition(jointId, &lastRef) && std::abs(lastRef - initialPos) < std::abs(target - initialPos))
    {
        yarp::os::SystemClock::delaySystem(0.01);
    }

    syncPort.interrupt();
    syncPort.close();

    return 0;
}
