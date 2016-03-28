// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CheckCanBus.hpp"
// --
#include <string>
#include <iostream>
#include <sstream>
// --

namespace teo
{

/************************************************************************/
CheckCanBus::CheckCanBus() { }

/************************************************************************/
// -- Función principal: Se llama en mod.runModule(rf) desde el main.cpp
bool CheckCanBus::configure(yarp::os::ResourceFinder &rf) {

    // -- TimeOut por defecto
    timeOut = 0;

    if(rf.check("help")) {
        printf("CheckCanBus options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        CD_DEBUG_NO_HEADER("%s\n",rf.toString().c_str());
        return false;
    }

    // -- Parametro: --timeout [s]
    if(rf.check("timeout")){
        timeOut = rf.find("timeOut").asInt(); // -- recoge el parametro de timeout
    }

    // -- Parametro: --ids
    if(rf.check("ids")){
        yarp::os::Bottle jointsCan0 = rf.findGroup("ids");  // -- Introduce en un objeto bottle el parámetro ids
        std::string strIds = jointsCan0.get(1).toString().c_str(); // -- strIds almacena los Ids que queremos comprobar
        std::stringstream streamIds(strIds); // --  tratamos el string de IDs como un stream llamado streamIds
        int n;
        while(streamIds>>n){
               std::cout<<n<<std::endl;
               vectorIds.push_back(n); // -- introduce en el vector los IDs
           }

        /* -- Comprobacion de que se introducen correctamente en un vector
        for(int i = 0; i < vectorIds.size(); i++)
            {
            printf("ID: %i\n", vectorIds.at(i));
        }
        */
    }

    // -- leemos del fichero (ex: rightArm.ini)
    //yarp::os::Bottle jointsCan0 = rf.findGroup("jointsCan0");
    //CD_DEBUG("%s\n",jointsCan0.toString().c_str());

    //-- /dev/can0 --
    /*
    yarp::os::Bottle devCan0 = rf.findGroup("devCan0");
    CD_DEBUG("%s\n",devCan0.toString().c_str());
    yarp::os::Property optionsDevCan0;
    optionsDevCan0.fromString(devCan0.toString());
    //Appended mode option for optionsDevCan0 (for --mode flag)
    optionsDevCan0.put("mode", mode);
    deviceDevCan0.open(optionsDevCan0);
    if (!deviceDevCan0.isValid()) {
        CD_ERROR("deviceDevCan0 instantiation not worked.\n");
        return false;
    }
    */

    CD_DEBUG("%s\n",rf.toString().c_str()); // -- nos muestra el contenido del objeto resource finder
    deviceDevCan0.open(rf);                 // -- Abre el dispositivo HicoCan (tarjeta) y le pasa el ResourceFinder
    if (!deviceDevCan0.isValid()) {
        CD_ERROR("deviceDevCan0 instantiation not worked.\n");
        return false;
    }
    deviceDevCan0.view(iCanBus); // -- ????????

    lastNow = yarp::os::Time::now(); // -- tiempo actual

    return this->start(); // arranca el hilo
}

/************************************************************************/
// -- ???????????????????????????????
bool CheckCanBus::updateModule() {
    //printf("CheckCanBus alive...\n");
    return true;
}

/************************************************************************/
// -- Para el hilo y para el dispositivo PolyDriver
bool CheckCanBus::close() {
    this->stop();

    deviceDevCan0.close();

    return true;
}

/************************************************************************/
// -- Función que lee los mensajes que le llegan del CAN-BUS
std::string CheckCanBus::msgToStr(can_msg* message) {

    std::stringstream tmp; // -- nos permite insertar cualquier tipo de dato dentro del flujo
    for(int i=0; i < message->dlc-1; i++)
    {
        tmp << std::hex << static_cast<int>(message->data[i]) << " ";
    }
    tmp << std::hex << static_cast<int>(message->data[message->dlc-1]);
    tmp << ". canId(";
    tmp << std::dec << (message->id & 0x7F); // -- muestra en decimal el ID
    tmp << ") via(";
    tmp << std::hex << (message->id & 0xFF80); // -- ???
    tmp << "), t:" << yarp::os::Time::now() - lastNow << "[s]."; // -- tiempo que ha tardado en responder???

    lastNow = yarp::os::Time::now();

    return tmp.str();
}

/*************************************************************************/

// -- Función que comprueba los mensajes que recibe del CAN con el vector de IDs
std::string CheckIds::msgToStr(can_msg* message) {

    std::stringstream tmp;
    for(int i=0; i < message->dlc-1; i++)
    {
        tmp << std::hex << static_cast<int>(message->data[i]) << " ";
    }
    tmp << std::hex << static_cast<int>(message->data[message->dlc-1]);
    tmp << ". canId(";
    tmp << std::dec << (message->id & 0x7F); // -- muestra en decimal el ID
    tmp << ") via(";
    tmp << std::hex << (message->id & 0xFF80); // -- ???
    tmp << "), t:" << yarp::os::Time::now() - lastNow << "[s]."; // -- tiempo que ha tardado en responder???

    lastNow = yarp::os::Time::now();

    return tmp.str();
}


/************************************************************************/

}  // namespace teo
