// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup examplePositionDirectOnlinePullLocal examplePositionDirectOnlinePullLocal
 * @brief This example connects to a local controlboard device and sends position direct commands.
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
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>

#include <StateObserver.hpp>

#define DEFAULT_REMOTE "/teo/leftArm"
#define DEFAULT_JOINT 5
#define DEFAULT_SPEED 2.0 // deg/s
#define DEFAULT_TARGET (-20.0)

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

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

    roboticslab::StateObserver syncObserver(1.0);

    yarp::os::Property options;
    options.fromString(rf.toString());
    options.put("syncObserver", yarp::os::Value::makeBlob(&syncObserver, sizeof(syncObserver)));

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

    const double velocity = std::copysign(speed, target - initialPos);
    const double offset = yarp::os::SystemClock::nowSystem();

    double lastRef;

    while (posd->getRefPosition(jointId, &lastRef) && std::abs(lastRef - initialPos) < std::abs(target - initialPos))
    {
        if (!syncObserver.await())
        {
            yWarning() << "Sync observer timeout";
            break;
        }

        double t = yarp::os::SystemClock::nowSystem() - offset;
        double e = initialPos + velocity * t;
        posd->setPosition(jointId, e);
    }

    return 0;
}
