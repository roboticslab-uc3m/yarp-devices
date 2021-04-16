// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup examplePositionDirect examplePositionDirect
 * @brief This example connects to a remote controlboard device and sends position direct commands.
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
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/dev/PolyDriver.h>

#define DEFAULT_REMOTE "/teo/leftArm"
#define DEFAULT_JOINT 5
#define DEFAULT_SPEED 2.0 // deg/s
#define DEFAULT_TARGET (-20.0)
#define DEFAULT_PERIOD_MS 50
#define DEFAULT_IP_MODE "pt"

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

        if (std::abs(distance) - std::abs(current - initial) < 1e-6)
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
    auto jointId = rf.check("id", yarp::os::Value(DEFAULT_JOINT), "joint id").asInt32();
    auto speed = rf.check("speed", yarp::os::Value(DEFAULT_SPEED), "trajectory speed (deg/s)").asFloat64();
    auto target = rf.check("target", yarp::os::Value(DEFAULT_TARGET), "target position (deg)").asFloat64();
    auto period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD_MS), "command period (ms)").asInt32() * 0.001;
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
    yarp::dev::IPositionDirect * posd;
    yarp::dev::IRemoteVariables * var;

    if (!dd.view(mode) || !dd.view(enc) || !dd.view(posd) || (rf.check("ip") && !dd.view(var)))
    {
        yError() << "Problems acquiring robot interfaces";
        return 1;
    }

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
        yError() << "Unable to set position direct mode";
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

    yInfo() << "Moving joint" << jointId << "to" << target << "degrees...";

    const double distance = target - initialPos;
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

        double lastRef;

        do
        {
            std::cout << "." << std::flush;
            yarp::os::SystemClock::delaySystem(0.5);
        }
        while (posd->getRefPosition(jointId, &lastRef) && std::abs(lastRef - target) > 1e-6);

        std::cout << " end" << std::endl;
    }
    else
    {
        Worker worker(period, initialPos, increment, distance, [=](auto pos) { posd->setPosition(jointId, pos); });

        if (!worker.start())
        {
            yError() << "Unable to start trajectory thread";
            return 1;
        }

        while (worker.isRunning())
        {
            yarp::os::SystemClock::delaySystem(0.1);
        }
    }

    return 0;
}
