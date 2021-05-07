// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
 * @defgroup exampleOnlineTrajectoryRemotePull exampleOnlineTrajectoryRemotePull
 * @brief Perform an online trajectory via position commands attending a remote callback.
 * @see @ref exampleOnlineTrajectoryRemotePull.cpp
 */

/**
 * @example{lineno} exampleOnlineTrajectoryRemotePull.cpp
 *
 * A constant-velocity, single-joint trajectory is generated with configurable final target and
 * motion speed. The period between consecutive points is fixed (maps to cyclic synchronous
 * position mode on the real robot, a.k.a. CSP). A callback is registered for listening to a
 * remote synchronization port, position commands will be prepared and sent in response.
 * Although prepared for remote execution, this application could be rewritten to connect to
 * a local instance of @ref CanBusControlboard; however, a better alternative exists (see notes).
 * The techniques showcased here are especially suited for online-generated trajectories, e.g.
 * joystick teleoperation of visual servoing.
 *
 * Usage (showing default option values):
@verbatim
 exampleOnlineTrajectoryRemotePull --robot /teo --part /leftArm --joint 5 --speed 2.0 --target -20.0
@endverbatim
 *
 * @see @ref exampleOnlineTrajectoryLocalPull.cpp Command a local instance of the real robot controller via
 * callback.
 * @see @ref exampleOnlineTrajectoryRemotePush.cpp Command a remote robot via direct position commands.
 * @see @ref exampleOnlineTrajectoryRemotePull.py
 * @see [Tutorial: Trajectory Execution](https://roboticslab-uc3m.github.io/teo-developer-manual/tutorial/trajectories.html) (external)
 * @note If you need to command a simulated robot, you must emulate the periodic synchronization
 * port via `yarp clock --name /teoSim/sync:o --period XXX` with the desired period between points
 * (in milliseconds) and assuming `--robot /teoSim` was passed in this example.
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
            double currentTime = b.get(0).asInt32() + b.get(1).asInt32() * 1e-9;

            if (count == 0)
            {
                offset = currentTime;
            }

            double t = currentTime - offset;
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
    auto joint = rf.check("joint", yarp::os::Value(DEFAULT_JOINT), "joint id").asInt32();
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
        yError() << "Unable to acquire robot interfaces";
        return 1;
    }

    yarp::os::BufferedPort<yarp::os::Bottle> syncPort;
    syncPort.setReadOnly();

    if (!syncPort.open("/exampleOnlineTrajectoryRemotePull/sync:i"))
    {
        yError() << "Unable to open local sync port";
        return 1;
    }

    if (!yarp::os::Network::connect(robot + "/sync:o", syncPort.getName(), "fast_tcp"))
    {
        yError() << "Unable to connect to remote sync port";
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
    double velocity = std::copysign(speed, distance);

    SyncCallback callback(initialPos, velocity, [=](auto pos) { posd->setPosition(joint, pos); });
    syncPort.useCallback(callback);

    double lastRef;

    while (posd->getRefPosition(joint, &lastRef) && std::abs(lastRef - initialPos) < std::abs(distance))
    {
        yarp::os::SystemClock::delaySystem(0.01);
    }

    syncPort.interrupt();
    syncPort.close();

    return 0;
}
