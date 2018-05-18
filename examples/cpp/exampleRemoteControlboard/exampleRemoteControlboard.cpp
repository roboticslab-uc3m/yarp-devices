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

#include <stdio.h>
#include <stdlib.h>

#include <vector>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

using namespace yarp::os;
using namespace yarp::dev;

int main(int argc, char *argv[]) {

    printf("WARNING: requires a running instance of RaveBot (i.e. testRaveBot or cartesianServer)\n");
    Network yarp;
    if (!Network::checkNetwork()) {
        printf("Please start a yarp name server first\n");
        return(-1);
    }
    Property options;
    options.put("device","remote_controlboard");
    options.put("remote","/ravebot");
    options.put("local","/local");

    PolyDriver dd(options);
    if(!dd.isValid()) {
      printf("RaveBot device not available.\n");
	  dd.close();
      Network::fini();
      return 1;
    }

    IPositionControl2 *pos;
    IVelocityControl2 *vel;
    IEncodersTimed *enc;
    IControlMode2 *mode;

    bool ok = true;
    ok &= dd.view(pos);
    ok &= dd.view(vel);
    ok &= dd.view(enc);
    ok &= dd.view(mode);

    if (!ok) {
        printf("[warning] Problems acquiring robot interface\n");
        return false;
    } else printf("[success] testAsibot acquired robot interface\n");

    int axes;
    pos->getAxes(&axes);

    std::vector<int> posModes(axes, VOCAB_CM_POSITION);
    mode->setControlModes(posModes.data());

    printf("test positionMove(1,35)\n");
    pos->positionMove(1, 35);

    printf("Delaying 5 seconds...\n");
    Time::delay(5);

    std::vector<int> velModes(axes, VOCAB_CM_VELOCITY);
    mode->setControlModes(velModes.data());

    printf("test velocityMove(0,10)\n");
    vel->velocityMove(0,10);

    printf("Delaying 5 seconds...\n");
    Time::delay(5);

    vel->velocityMove(0,0); // stop the robot

    return 0;
}
