// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "OneTechnosoftIpos.hpp"

namespace teo
{

/************************************************************************/
OneTechnosoftIpos::OneTechnosoftIpos() { }

/************************************************************************/
bool OneTechnosoftIpos::configure(ResourceFinder &rf) {

    if(rf.check("help")) {
        printf("OneTechnosoftIpos options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        CD_DEBUG_NO_HEADER("%s\n",rf.toString().c_str());
        return false;
    }

    Property canBusOptions;
    canBusOptions.put("device","CanBusHico");
    canBusOptions.put("canDevice","/dev/can1");
    canBusOptions.put("canBitrate",BITRATE_1000k);
    canBusDevice.open(canBusOptions);
    if( ! canBusDevice.isValid() ){
        CD_ERROR("canBusDevice instantiation not worked.\n");
        return false;
    }
    canBusDevice.view(iCanBus);

    yarp::os::Property options;
    options.put("device","TechnosoftIpos");
    options.put("canId",10);  // locomotion 10 on /dev/can1 = left knee.
    options.put("tr",235.2);
    options.put("min",-5);
    options.put("max",80);
    options.put("refAcceleration",0.575437);
    options.put("refSpeed",737.2798);
    dd.open(options);
    if(!dd.isValid()) {
      printf("TechnosoftIpos device not available.\n");
      dd.close();
      yarp::os::Network::fini();
      return false;
    }

    //-- View TechnosoftIpos interfaces.
    bool ok = dd.view(iCanBusSharer);
    if (!ok) {
        printf("[error] Problems viewing ICanBusSharer.\n");
        return 1;
    } else printf("[success] Viewing ICanBusSharer.\n");

    ok = dd.view(pos);
    if (!ok) {
        printf("[error] Problems viewing IPositionControlRaw.\n");
        return 1;
    } else printf("[success] Viewing IPositionControlRaw.\n");

    ok = dd.view(enc);
    if (!ok) {
        printf("[error] Problems viewing IEncodersRaw.\n");
        return 1;
    } else printf("[success] Viewing IEncodersRaw.\n");

    ok = dd.view(vel);
    if (!ok) {
        printf("[error] Problems viewing IVelocityControlRaw.\n");
        return 1;
    } else printf("[success] Viewing IVelocityControlRaw.\n");

    ok = dd.view(ctrl);
    if (!ok) {
        printf("[error] Problems viewing IControlModeRaw.\n");
        return 1;
    } else printf("[success] Viewing IControlModeRaw.\n");

    //-- Pass before sending commands.
    iCanBusSharer->setCanBusPtr(iCanBus);

    int got;
    ctrl->getControlModeRaw(0,&got);

    struct can_msg buffer;
    while( iCanBus->read_timeout(&buffer,1) <= 0 );
    printf("Read CAN message\n");
    iCanBusSharer->interpretMessage(&buffer);

    /*iCanBusSharer->start();

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

    printf("test positionMove(0,8)\n");
    ok = pos->positionMoveRaw(0,8);
    if (!ok) {
        printf("[error] Problems in positionMove.\n");
        return 1;
    } else printf("[success] positionMove.\n");*/

    printf("Please quit with ^C\n");

    return true;
}

/************************************************************************/

bool OneTechnosoftIpos::updateModule() {
    //printf("OneTechnosoftIpos alive...\n");
    return true;
}

/************************************************************************/

bool OneTechnosoftIpos::close() {
    dd.close();

    return true;
}

/************************************************************************/

}  // namespace teo
