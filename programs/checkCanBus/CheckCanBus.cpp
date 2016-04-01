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
    timeOut = 0;    // -- Por defecto 0 [s] el parámetro --timeout
    firstTime = 0;  // -- inicializo el tiempo a 0 [s] la primera vez que arranca el programa


    if(rf.check("help")) {
        printf("CheckCanBus options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        CD_DEBUG_NO_HEADER("%s\n",rf.toString().c_str());
        return false;
    }

    // -- Parametro: --timeout [s]
    if(rf.check("timeOut")){
        timeOut = rf.find("timeOut").asInt();

        printf("[INFO] Timeout: %.2f [s]\n", timeOut);
    }

    // -- Parametro: --ids (introduce los IDs en una cola)
    if(rf.check("ids")){
        yarp::os::Bottle jointsCan0 = rf.findGroup("ids");  // -- Introduce en un objeto bottle el parámetro ids
        std::string strIds = jointsCan0.get(1).toString(); // -- strIds almacena los Ids que queremos comprobar
        std::stringstream streamIds(strIds); // --  tratamos el string de IDs como un stream llamado streamIds
        CD_INFO_NO_HEADER("[INFO] Se va a proceder a la detección de los IDs: ");
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
        CD_INFO_NO_HEADER("[INFO] Se va a proceder a la detección de los IDs: ");
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
    tmp << "), t:" << yarp::os::Time::now() - lastNow << "[s]."; // -- diferencia entre el tiempo actual y el del último mensaje

    lastNow = yarp::os::Time::now();    // -- tiempo actual (aleatorio)

    return tmp.str(); // -- devuelve el string (mensaje)
}

/*************************************************************************/

// -- Función que comprueba los mensajes que recibe del CAN utilizando una cola de IDs
void CheckCanBus::checkIds(can_msg* message) {
    for(int i=0; i<queueIds.size(); i++){  // -- bucle que recorrerá la cola

        if(queueIds.front()== (message->id & 0x7F)) {   // -- si el ID coincide, lo saco de la cola
            CD_SUCCESS_NO_HEADER("Se ha detectado el ID: %i\n", queueIds.front());
            queueIds.pop(); // -- saca de la cola el elemento
        }
        // En caso de que no coincida el ID
        else{
           int res = queueIds.front(); // -- residuo que volveriamos a introducir en la cola
           queueIds.pop();      // -- saca de la cola el primer elemento
           queueIds.push(res);  // -- lo vuelve a introducir al final
        }
    }
}

// -- Función que imprime por pantalla los IDs no detectados (IDs residuales en cola)
void CheckCanBus::printWronglIds(){
    for(int i=0; i<queueIds.size(); i++){
               CD_ERROR_NO_HEADER("No se ha detectado el ID: %i\n", queueIds.front());
               queueIds.pop(); // -- saca de la cola el elemento
           }       
}

/************************************************************************/

}  // namespace teo
