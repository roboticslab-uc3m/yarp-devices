// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup asibot_examples_cpp
 * \defgroup testRemoteRaveBot testRemoteRaveBot
 *
 * @brief This example connects to a running \ref testRaveBot or \ref cartesianServer module.
 *
 * <b>Legal</b>
 *
 * Copyright: (C) 2010 Universidad Carlos III de Madrid;
 *            (C) 2010 RobotCub Consortium
 *
 * Author: Juan G Victores
 *
 * Contribs: Paul Fitzpatrick and Giacomo Spigler (YARP dev/motortest.cpp example)
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * <b>Building</b>
\verbatim
cd repos/asibot-main/example/cpp
mkdir build; cd build; cmake ..
make -j3
\endverbatim
 *
 * <b>Running</b>
\verbatim
./testRemoteRaveBot
\endverbatim
 *
 */

#include <cstdio>

#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>

int main(int argc, char *argv[])
{
    std::printf("WARNING: requires a running instance of RaveBot (i.e. testRaveBot or cartesianServer)\n");
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        std::printf("Please start a yarp name server first\n");
        return 1;
    }

    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/ravebot");
    options.put("local", "/local");

    yarp::dev::PolyDriver dd(options);

    if (!dd.isValid())
    {
        std::printf("RaveBot device not available.\n");
        return 1;
    }

    yarp::dev::IPositionControl *pos;
    yarp::dev::IVelocityControl *vel;
    yarp::dev::IEncodersTimed *enc;
    yarp::dev::IControlMode *mode;

    bool ok = true;
    ok &= dd.view(pos);
    ok &= dd.view(vel);
    ok &= dd.view(enc);
    ok &= dd.view(mode);

    if (!ok)
    {
        std::printf("[warning] Problems acquiring robot interface\n");
        return 1;
    } else std::printf("[success] testAsibot acquired robot interface\n");

    int axes;
    pos->getAxes(&axes);

    std::vector<int> posModes(axes, VOCAB_CM_POSITION);
    mode->setControlModes(posModes.data());

    std::printf("test positionMove(1, 35.0)\n");
    pos->positionMove(1, 35.0);

    std::printf("Delaying 5 seconds...\n");
    yarp::os::Time::delay(5.0);

    std::vector<int> velModes(axes, VOCAB_CM_VELOCITY);
    mode->setControlModes(velModes.data());

    std::printf("test velocityMove(0, 10.0)\n");
    vel->velocityMove(0, 10.0);

    std::printf("Delaying 5 seconds...\n");
    yarp::os::Time::delay(5.0);

    vel->velocityMove(0, 0.0); // stop the robot

    return 0;
}
