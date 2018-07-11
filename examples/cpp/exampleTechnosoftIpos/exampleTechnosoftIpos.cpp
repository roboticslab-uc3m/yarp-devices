// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_examples_cpp
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

#include <cstdio>
#include <stdlib.h>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CanBusInterface.h>

#include <ICanBusSharer.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (! yarp::os::Network::checkNetwork() )
    {
        std::printf("Please start a yarp name server first\n");
        return 1;
    }

    yarp::dev::PolyDriver canBusDevice;
    yarp::dev::ICanBus* iCanBus;
    yarp::dev::ICanBufferFactory* iCanBufferFactory;

    yarp::os::Property canBusOptions;
    canBusOptions.put("device","CanBusHico");
    canBusOptions.put("canDevice","/dev/can1");
    canBusOptions.put("canBitrate",8);
    canBusDevice.open(canBusOptions);
    if( ! canBusDevice.isValid() )
    {
        std::printf("canBusDevice instantiation not worked.\n");
        return 1;
    }
    canBusDevice.view(iCanBus);
    canBusDevice.view(iCanBufferFactory);

    yarp::os::Property options;
    options.put("device","TechnosoftIpos");
    options.put("canId",23);
    options.put("tr",120);
    options.put("min",-60);
    options.put("max",60);
    options.put("refAcceleration",0.575437);
    options.put("refSpeed",737.2798);

    yarp::os::Value v(&iCanBufferFactory, sizeof(iCanBufferFactory));
    options.put("canBufferFactory", v);

    yarp::dev::PolyDriver dd(options);
    if(!dd.isValid())
    {
        std::printf("TechnosoftIpos device not available.\n");
        dd.close();
        yarp::os::Network::fini();
        return 1;
    }

    //-- View TechnosoftIpos interfaces.
    roboticslab::ICanBusSharer *iCanBusSharer;
    bool ok = dd.view(iCanBusSharer);
    if (!ok)
    {
        std::printf("[error] Problems viewing ICanBusSharer.\n");
        return 1;
    }
    else std::printf("[success] Viewing ICanBusSharer.\n");

    yarp::dev::IPositionControlRaw *pos;
    ok = dd.view(pos);
    if (!ok)
    {
        std::printf("[error] Problems viewing IPositionControlRaw.\n");
        return 1;
    }
    else std::printf("[success] Viewing IPositionControlRaw.\n");

    yarp::dev::IEncodersRaw *enc;
    ok = dd.view(enc);
    if (!ok)
    {
        std::printf("[error] Problems viewing IEncodersRaw.\n");
        return 1;
    }
    else std::printf("[success] Viewing IEncodersRaw.\n");

    yarp::dev::IVelocityControlRaw *vel;
    ok = dd.view(vel);
    if (!ok)
    {
        std::printf("[error] Problems viewing IVelocityControlRaw.\n");
        return 1;
    }
    else std::printf("[success] Viewing IVelocityControlRaw.\n");

    yarp::dev::IControlMode2Raw *mode;
    ok = dd.view(mode);
    if (!ok)
    {
        std::printf("[error] Problems viewing IControlModeRaw.\n");
        return 1;
    }
    else std::printf("[success] Viewing IControlModeRaw.\n");

    //-- Pass before sending commands.
    iCanBusSharer->setCanBusPtr(iCanBus);

    iCanBusSharer->start();

    yarp::os::Time::delay(0.1);
    iCanBusSharer->readyToSwitchOn();

    yarp::os::Time::delay(0.1);
    iCanBusSharer->switchOn();

    yarp::os::Time::delay(2);
    iCanBusSharer->enable();

    //-- Commands on TechnosoftIpos.
    ok = mode->setControlModeRaw(0, VOCAB_CM_POSITION);
    if (!ok)
    {
        std::printf("[error] Problems in setPositionModeRaw.\n");
        return 1;
    }
    else std::printf("[success] setPositionModeRaw.\n");

    std::printf("test positionMove(0,-25)\n");
    ok = pos->positionMoveRaw(0, -25);
    if (!ok)
    {
        std::printf("[error] Problems in positionMove.\n");
        return 1;
    }
    else std::printf("[success] positionMove.\n");

    std::printf("Delaying 5 seconds...\n");
    yarp::os::Time::delay(5);

    /*vel->setVelocityModeRaw();
    printf("test velocityMove(0,10)\n");
    vel->velocityMoveRaw(0,10);

    printf("Delaying 5 seconds...\n");
    yarp::os::Time::delay(5);*/

    dd.close();

    return 0;
}
