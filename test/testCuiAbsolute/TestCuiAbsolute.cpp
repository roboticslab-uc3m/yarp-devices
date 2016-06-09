// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TestCuiAbsolute.hpp"
// --
#include <string>
#include <iostream>
#include <sstream>


YARP_DECLARE_PLUGINS(BodyYarp)

namespace teo
{

/************************************************************************/
TestCuiAbsolute::TestCuiAbsolute() {}



/************************************************************************/
// -- Función principal: Se llama en mod.runModule(rf) desde el main.cpp
bool TestCuiAbsolute::configure(yarp::os::ResourceFinder &rf) {

    firstTime = 0;      // -- inicializo el tiempo a 0 [s] la primera vez que arranca el programa
    cleaningTime = 1;   // -- Por defecto 1 [s] el parámetro --cleaningTime

    // -- Antes de configurar los periféricos, check a parámetro --help
    if(rf.check("help")) {
        printf("TestCuiAbsolute options:\n");
        printf("\t--help (this help)\t\t --from [file.ini]\t --context [path]*\t --id[ID of Cui Absolute]\n");       
        printf("Modes: \t--startContinuousPublishing\t--startPullPublishing\t--stopPublishing\n");
        printf("*can0: \t--canDevice /dev/can0\t\t can1: --canDevice /dev/can1\n");
        CD_DEBUG_NO_HEADER("%s\n",rf.toString().c_str());
        ::exit(1);
        return false;
    }

    // -- Parametro: --id (ID del encoder absoluto al que vamos a enviar los mensajes)
        if(rf.check("id")){
            id = rf.find("id").asInt();
        }

    // -- Continuación del código que CONFIGURA LA HICO-CAN y LOS DRIVERS
    CD_DEBUG("%s\n",rf.toString().c_str()); // -- nos muestra el contenido del objeto resource finder
    deviceDevCan0.open(rf);                 // -- Abre el dispositivo HicoCan (tarjeta) y le pasa el ResourceFinder
    if (!deviceDevCan0.isValid()) {
        CD_ERROR("deviceDevCan0 instantiation not worked.\n");
        return false;
    }
    deviceDevCan0.view(iCanBus);            // -- conecta el dispositivo (hicocan)

    // ---------- adding configuration of Cui Absolute Encoders (se trata de la configuración minima que necesita el encoder)
    std::stringstream strconf;
    strconf << "(device CuiAbsolute) (canId " << id << ") (min 0) (max 0) (tr 1) (refAcceleration 0.0) (refSpeed 0.0)";    
    CD_DEBUG("%s\n",strconf.str().c_str());
    yarp::os::Property CuiAbsoluteConf (strconf.str().c_str());

    bool cuiOk = true;

    cuiOk &= canNodeCuiAbsolute.open( CuiAbsoluteConf );
    cuiOk &= canNodeCuiAbsolute.view( iControlLimitsRaw  );
    cuiOk &= canNodeCuiAbsolute.view( iControlModeRaw );
    cuiOk &= canNodeCuiAbsolute.view( iEncodersTimedRaw );
    cuiOk &= canNodeCuiAbsolute.view( iPositionControlRaw );
    cuiOk &= canNodeCuiAbsolute.view( iPositionDirectRaw );
    cuiOk &= canNodeCuiAbsolute.view( iTorqueControlRaw );
    cuiOk &= canNodeCuiAbsolute.view( iVelocityControlRaw );
    cuiOk &= canNodeCuiAbsolute.view( iCanBusSharer );
    cuiOk &= canNodeCuiAbsolute.view( cuiAbsoluteEncoder ); // -- conecta el dispositivo (encoders absolutos)

    // --Checking Cui Absolute Encoders
    if(cuiOk){
        CD_SUCCESS("Configuration of CuiAbsolute sucessfully :)\n");
    }
    else{
        CD_ERROR("Bad Configuration of CuiAbsolute:(\n");
        ::exit(1);
    }

    //-- Pass CAN bus (HicoCAN) pointer to CAN node.
    iCanBusSharer->setCanBusPtr( iCanBus );


    // -- Parametro para PIC: --startContinuousPublishing
    if(rf.check("startContinuousPublishing")){
        CD_INFO("Start PIC of Cui publishing messages in continuous mode\n");
        // -- publishing PIC Cui messages        
        cuiAbsoluteEncoder->startContinuousPublishing(0);
        yarp::os::Time::delay(1);
        this->stopModule();
    }

    // -- Parametro para PIC: --startPullPublishing
    if(rf.check("startPullPublishing")){
        CD_INFO("Start PIC publishing messages in pulling mode\n");        
        // -- publishing PIC Cui messages
        cuiAbsoluteEncoder->startPullPublishing();
        yarp::os::Time::delay(1);
        this->stopModule();
    }

    // -- Parametro para PIC: --stopPublishing
    if(rf.check("stopPublishing")){
        CD_INFO("Stop PIC publishing messages\n");        
        // -- publishing PIC Cui messages
        cuiAbsoluteEncoder->stopPublishingMessages();
        yarp::os::Time::delay(1);
        this->stopModule();
    }

    // -- Control de errores
    if(!rf.check("startContinuousPublishing") && !rf.check("startPullPublishing") && !rf.check("stopPublishing")){
        CD_ERROR("You need to specify more parameters. You can use \"testCuiAbsolute --help\"\n");

        return false;
    }

    if(!rf.check("id")){
        CD_ERROR("You need to specify the ID of Cui Absolute encoder. Example: \"--id 124\"\n");
        return false;
    }

    return true;
}

/************************************************************************/
bool TestCuiAbsolute::updateModule() {
    //printf("TestCuiAbsolute alive...\n");
    return true;
}

/************************************************************************/
// -- Para el hilo y para el dispositivo PolyDriver
bool TestCuiAbsolute::close() {

    deviceDevCan0.close();

    return true;
}

}  // namespace teo
