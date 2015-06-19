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
    options.put("canId",23);
    options.put("tr",120);
    options.put("min",-60);
    options.put("max",60);
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
