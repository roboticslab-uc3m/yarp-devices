#include <string>
#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>

#include <ColorDebug.h>

#define DEFAULT_REMOTE "/teo/leftArm"
#define DEFAULT_JOINT 5
#define DEFAULT_POS_TARGET (-10.0)
#define DEFAULT_POSD_TARGET (-20.0)
#define DEFAULT_POSD_PERIOD_MS 50
#define DEFAULT_POSD_INCREMENT (-0.1)

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    std::string remote = rf.check("remote", yarp::os::Value(DEFAULT_REMOTE), "remote port").asString();
    int jointId = rf.check("id", yarp::os::Value(DEFAULT_JOINT), "joint id").asInt32();
    double posTarget = rf.check("posTarget", yarp::os::Value(DEFAULT_POS_TARGET), "target position for pos mode [deg]").asFloat64();
    double posdTarget = rf.check("posdTarget", yarp::os::Value(DEFAULT_POSD_TARGET), "target position for posd mode [deg]").asFloat64();
    int period = rf.check("period", yarp::os::Value(DEFAULT_POSD_PERIOD_MS), "posd command period [ms]").asInt32();
    double increment = rf.check("increment", yarp::os::Value(DEFAULT_POSD_INCREMENT), "posd command increment [deg]").asFloat64();

    if ((posdTarget - posTarget) * increment <= 0)
    {
        CD_ERROR("Illegal source and target positions with given increment.\n");
        return 1;
    }

    if (period <= 0)
    {
        CD_ERROR("Illegal period.\n");
        return false;
    }

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR("Please start a yarp name server first.\n");
        return 1;
    }

    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", remote);
    options.put("local", "/examplePositionDirect");

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
      CD_ERROR("Remote device not available.\n");
      return 1;
    }

    yarp::dev::IControlMode * mode;
    yarp::dev::IEncoders * enc;
    yarp::dev::IPositionControl * pos;
    yarp::dev::IPositionDirect * posd;

    bool ok = true;

    ok &= dd.view(mode);
    ok &= dd.view(enc);
    ok &= dd.view(pos);
    ok &= dd.view(posd);

    if (!ok)
    {
        CD_ERROR("Problems acquiring robot interfaces.\n");
        return 1;
    }

    int numJoints;
    enc->getAxes(&numJoints);

    if (jointId < 0 || jointId > numJoints - 1)
    {
        CD_ERROR("Illegal joint ID: %d (numJoints: %d).\n", jointId, numJoints);
        return 1;
    }

    CD_INFO("-- testing POSITION MODE --\n");
    std::vector<int> positionMode(numJoints, VOCAB_CM_POSITION);

    if (!mode->setControlModes(positionMode.data()))
    {
        CD_ERROR("Problems setting position control: POSITION.\n");
        return 1;
    }

    CD_INFO("Moving joint %d to %f degrees...\n", jointId, posTarget);
    pos->positionMove(jointId, posTarget);

    getchar();

    CD_INFO("-- testing POSITION DIRECT --\n");
    std::vector<int> positionDirectMode(numJoints, VOCAB_CM_POSITION_DIRECT);

    if (!mode->setControlModes(positionDirectMode.data()))
    {
        CD_ERROR("Problems setting position control: POSITION_DIRECT.\n");
        return 1;
    }

    double encValue;

    if (!enc->getEncoder(jointId, &encValue))
    {
        CD_ERROR("getEncoders() failed.\n");
        return 1;
    }

    CD_INFO("Current ENC value: %f\n", encValue);

    getchar();

    CD_INFO("Moving joint %d to %f degrees...\n", jointId, posdTarget);

    while ((posdTarget - encValue) * increment > 0.0)
    {
        encValue += increment;
        posd->setPosition(jointId, encValue);
        yarp::os::Time::delay(period * 0.001);
    }

    return 0;
}
