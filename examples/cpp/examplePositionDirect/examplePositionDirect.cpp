// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup examplePositionDirect examplePositionDirect
 * @brief This example connects to a remote controlboard device and sends position direct commands.
 */

#include <cmath>

#include <iostream>
#include <string>
#include <vector>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

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

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    std::string remote = rf.check("remote", yarp::os::Value(DEFAULT_REMOTE), "remote port").asString();
    int jointId = rf.check("id", yarp::os::Value(DEFAULT_JOINT), "joint id").asInt32();
    double speed = rf.check("speed", yarp::os::Value(DEFAULT_SPEED), "trajectory speed (def/s)").asFloat64();
    double posTarget = rf.check("posTarget", yarp::os::Value(DEFAULT_POS_TARGET), "target position for pos mode [deg]").asFloat64();
    double posdTarget = rf.check("posdTarget", yarp::os::Value(DEFAULT_POSD_TARGET), "target position for posd mode [deg]").asFloat64();
    int period = rf.check("period", yarp::os::Value(DEFAULT_POSD_PERIOD_MS), "posd command period [ms]").asInt32();
    bool batch = rf.check("batch", "stream interpolation data in batches");
    std::string ipMode = rf.check("ipMode", yarp::os::Value(DEFAULT_IP_MODE), "linear interpolation mode [pt|pvt]").asString();

    if (speed <= 0)
    {
        yError() << "Illegal speed:" << speed;
        return 1;
    }

    if (period <= 0)
    {
        yError() << "Illegal period:" << period;
        return false;
    }

    if (ipMode != "pt" && ipMode != "pvt")
    {
        yError() << "Illegal ipMode:" << ipMode;
        return false;
    }

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", remote);
    options.put("local", "/examplePositionDirect");

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

    bool ok = true;

    ok &= dd.view(mode);
    ok &= dd.view(enc);
    ok &= dd.view(pos);
    ok &= dd.view(posd);
    ok &= !batch || dd.view(var);

    if (!ok)
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
    pos->positionMove(jointId, posTarget);

    getchar();

    yInfo() << "-- testing POSITION DIRECT --";

    if (batch)
    {
        yarp::os::Property dict {{"enable", yarp::os::Value(true)},
                                 {"mode", yarp::os::Value(ipMode)},
                                 {"periodMs", yarp::os::Value(period)}};

        yarp::os::Value v;
        v.asList()->addString("linInterp");
        v.asList()->addList().fromString(dict.toString());

        if (!var->setRemoteVariable("all", {v}))
        {
            yError() << "Unable to set linear interpolation mode";
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

    getchar();

    yInfo() << "Moving joint" << jointId << "to" << posdTarget << "degrees...";

    const double distance = posdTarget - posTarget;
    const double increment = (speed * period * 0.001) / distance;
    double progress = 0.0;

    while (std::abs(progress) < 1.0)
    {
        progress += increment;
        double newPos = initialPos + progress * std::abs(distance);
        yInfo() << "New target:" << newPos;
        posd->setPosition(jointId, newPos);

        if (!batch)
        {
            yarp::os::Time::delay(period * 0.001);
        }
    }

    if (batch)
    {
        bool done = false;

        do
        {
            std::cout << ".";
            yarp::os::Time::delay(0.5);
        }
        while (pos->checkMotionDone(&done) && !done);

        std::cout << " end" << std::endl;
    }

    return 0;
}
