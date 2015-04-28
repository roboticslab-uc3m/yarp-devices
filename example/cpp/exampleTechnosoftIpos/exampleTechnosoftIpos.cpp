// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup teo_body_examples_cpp
 * \defgroup exampleTechnosoftIpos exampleTechnosoftIpos
 *
 * @brief This example.
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
cd example/cpp
mkdir build; cd build; cmake ..
make -j3
\endverbatim
 *
 * <b>Running</b>
\verbatim
./exampleTechnosoftIpos
\endverbatim
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "ICanBusSharer.h"

YARP_DECLARE_PLUGINS(BodyYarp)

int main(int argc, char *argv[]) {

    YARP_REGISTER_PLUGINS(BodyYarp);

    yarp::os::Network yarp;
    if (! yarp::os::Network::checkNetwork() ) {
        printf("Please start a yarp name server first\n");
        return 1;
    }

    teo::CanBusHico canBusHico;
    std::string canDevicePath = "/dev/can1";
    int canBitrate = BITRATE_1000k;
    //-- Initialize the CAN device (e.g. /dev/can1).
    if( ! canBusHico.init(canDevicePath, canBitrate) )
        return 1;

    yarp::os::Property options;
    options.put("device","TechnosoftIpos");
    options.put("canId",23);
    options.put("tr",120);
    yarp::dev::PolyDriver dd(options);
    if(!dd.isValid()) {
      printf("TechnosoftIpos device not available.\n");
	  dd.close();
      yarp::os::Network::fini();
      return 1;
    }

    //-- View TechnosoftIpos interfaces.
    teo::ICanBusSharer *iCanBusSharer;
    bool ok = dd.view(iCanBusSharer);
    if (!ok) {
        printf("[error] Problems viewing ICanBusSharer.\n");
        return 1;
    } else printf("[success] Viewing ICanBusSharer.\n");

    yarp::dev::IPositionControlRaw *pos;
    ok = dd.view(pos);
    if (!ok) {
        printf("[error] Problems viewing IPositionControlRaw.\n");
        return 1;
    } else printf("[success] Viewing IPositionControlRaw.\n");

    yarp::dev::IEncodersRaw *enc;
    ok = dd.view(enc);
    if (!ok) {
        printf("[error] Problems viewing IEncodersRaw.\n");
        return 1;
    } else printf("[success] Viewing IEncodersRaw.\n");

    yarp::dev::IVelocityControlRaw *vel;
    ok = dd.view(vel);
    if (!ok) {
        printf("[error] Problems viewing IVelocityControlRaw.\n");
        return 1;
    } else printf("[success] Viewing IVelocityControlRaw.\n");

    //-- Pass before sendong commands
    iCanBusSharer->setCanBusPtr(&canBusHico);

    iCanBusSharer->start();

    yarp::os::Time::delay(0.1);
    iCanBusSharer->readyToSwitchOn();

    yarp::os::Time::delay(0.1);
    iCanBusSharer->switchOn();

    yarp::os::Time::delay(2);
    iCanBusSharer->enable();

    //-- Commands on TechnosoftIpos.
    ok = pos->setPositionModeRaw();
    if (!ok) {
        printf("[error] Problems in setPositionModeRaw.\n");
        return 1;
    } else printf("[success] setPositionModeRaw.\n");

    printf("test positionMove(0,-25)\n");
    ok = pos->positionMoveRaw(0, -25);
    if (!ok) {
        printf("[error] Problems in positionMove.\n");
        return 1;
    } else printf("[success] positionMove.\n");

    printf("Delaying 5 seconds...\n");
    yarp::os::Time::delay(5);

    /*vel->setVelocityModeRaw();
    printf("test velocityMove(0,10)\n");
    vel->velocityMoveRaw(0,10);

    printf("Delaying 5 seconds...\n");
    yarp::os::Time::delay(5);*/

    return 0;
}


