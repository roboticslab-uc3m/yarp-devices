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

    timeOut = 2;        // -- Por defecto 2 [s] el parámetro --timeOut
    firstTime = 0;      // -- inicializo el tiempo a 0 [s] la primera vez que arranca el programa
    cleaningTime = 1;   // -- Por defecto 1 [s] el parámetro --cleaningTime


    if(rf.check("help")) {
        printf("CheckCanBus options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\t --timeOut [s]\t --cleaningTime [s]\n");
        CD_DEBUG_NO_HEADER("%s\n",rf.toString().c_str());
        return false;
    }

    // -- Parametro: --timeout [s]
    if(rf.check("timeOut")){
        timeOut = rf.find("timeOut").asInt();
        printf("[INFO] Timeout: %.2f [s]\n", timeOut);
    }

    // -- Parametro: --resetNodes
    if(rf.check("resetNodes")){
        printf("[INFO] Reseting all nodes\n");

    }

    // -- Parametro: --cleaningTime [s]
    if(rf.check("cleaningTime")){
        cleaningTime = rf.find("cleaningTime").asInt();
        printf("[INFO] Cleaning Time: %.2f [s]\n", cleaningTime);
    }

    // -- Parametro: --ids (introduce los IDs en una cola)
    if(rf.check("ids")){
        yarp::os::Bottle jointsCan0 = rf.findGroup("ids");  // -- Introduce en un objeto bottle el parámetro ids
        std::string strIds = jointsCan0.get(1).toString(); // -- strIds almacena los Ids que queremos comprobar
        std::stringstream streamIds(strIds); // --  tratamos el string de IDs como un stream llamado streamIds
        CD_INFO_NO_HEADER("[INFO] It will proceed to detect IDs: ");
        int n;
        while(streamIds>>n){    // -- recorre el stream y va introduciendo cada ID en la cola
               printf("%i ",n);
               queueIds.push(n); // -- introduce en la cola los IDs
           }
        printf("\n");
    }

    /*  -- Parametro: --from (con este parámetro indicamos el lugar del .ini donde especificamos la configuración+
     *  de la HicoCan y las ids que queremos comprobar)
     */
    else{
        yarp::os::Bottle fileIds = rf.findGroup("ids");
        std::string strIds = fileIds.get(1).toString(); // -- strIds almacena los Ids que queremos comprobar
        std::stringstream streamIds(strIds); // --  tratamos el string de IDs como un stream llamado streamIds
        CD_INFO_NO_HEADER("[INFO] It will proceed to detect IDs: ");
        int n;
        while(streamIds>>n){    // -- recorre el stream y va introduciendo cada ID en la cola
               printf("%i ",n);
               queueIds.push(n); // -- introduce en la cola los IDs
           }
        printf("\n");
    }

    // -- Continuación del código que CONFIGURA LA HICO-CAN
    CD_DEBUG("%s\n",rf.toString().c_str()); // -- nos muestra el contenido del objeto resource finder
    deviceDevCan0.open(rf);                 // -- Abre el dispositivo HicoCan (tarjeta) y le pasa el ResourceFinder
    if (!deviceDevCan0.isValid()) {
        CD_ERROR("deviceDevCan0 instantiation not worked.\n");
        return false;
    }
    deviceDevCan0.view(iCanBus);
    lastNow = yarp::os::Time::now(); // -- tiempo actual

    return this->start(); // arranca el hilo
}

/************************************************************************/
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
    tmp << "), t:" << yarp::os::Time::now() - lastNow << "[s]."; // -- diferencia entre el tiempo actual y el del último mensaje

    lastNow = yarp::os::Time::now();    // -- tiempo actual (aleatorio)

    return tmp.str(); // -- devuelve el string (mensaje)
}

/*************************************************************************/

// -- Función que comprueba los mensajes que recibe del CAN utilizando una cola de IDs
void CheckCanBus::checkIds(can_msg* message) {
    // -- Almacenamos el contenido del mensaje en un stream
    std::stringstream tmp; // -- stream que almacenará el mensaje recibido
    for(int i=0; i < message->dlc-1; i++)
        tmp << std::hex << static_cast<int>(message->data[i]) << " ";   // -- inserta los bytes del mensaje menos el último
    tmp << std::hex << static_cast<int>(message->data[message->dlc-1]); // -- inserta último byte

    // -- Recorremos la cola en busca del ID que ha lanzado el CAN
    for(int i=0; i<queueIds.size(); i++){  // -- bucle que recorrerá la cola

        if(queueIds.front()== (message->id & 0x7F)) {   // -- si el ID coincide, lo saco de la cola
            CD_SUCCESS_NO_HEADER("Detected ID: %i\n", queueIds.front());
            if(!tmp.str().compare("80 85 1 0 0 0 0 0")) CD_WARNING_NO_HEADER("[WARNING] Detected possible cleaning of driver settings: %i\n ", queueIds.front());
            queueIds.pop(); // -- saca de la cola el elemento
        }
        else{                                           // -- En caso de que no coincida el ID
           int res = queueIds.front(); // -- residuo que volveriamos a introducir en la cola
           queueIds.pop();      // -- saca de la cola el primer elemento
           queueIds.push(res);  // -- lo vuelve a introducir al final
        }
    }
}

// -- Función que imprime por pantalla los IDs no detectados (IDs residuales en cola)
void CheckCanBus::printWronglIds(){
    for(int i=0; i<queueIds.size(); i++){
               CD_ERROR_NO_HEADER("Has not been detected ID: %i\n", queueIds.front());
               queueIds.pop(); // -- saca de la cola el elemento
           }       
}

/************************************************************************/

}  // namespace teo
