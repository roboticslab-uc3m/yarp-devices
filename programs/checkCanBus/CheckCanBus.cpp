// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CheckCanBus.hpp"

namespace teo
{

/************************************************************************/
CheckCanBus::CheckCanBus() { }

/************************************************************************/
// -- Función principal (¿dónde se llama?)
bool CheckCanBus::configure(yarp::os::ResourceFinder &rf) {

    // -- TimeOut por defecto
    timeOut = 0;



    if(rf.check("help")) {
        printf("CheckCanBus options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        CD_DEBUG_NO_HEADER("%s\n",rf.toString().c_str());
        return false;
    }

    if(rf.check("timeout")){
        timeOut = rf.find("timeOut").asInt(); // -- recoge el parametro de timeout
    }


    CD_DEBUG("%s\n",rf.toString().c_str()); // -- nos muestra el contenido del objeto resource finder
    deviceDevCan0.open(rf); // -- Abre un dispositivo pasandole el contenido del RF (???????????)
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
