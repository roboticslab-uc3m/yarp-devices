// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CheckCanBus.hpp"

#include <cstdio>
#include <cstdlib>

#include <ios>
#include <string>
#include <sstream>

#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>

#include <ColorDebug.h>

namespace roboticslab
{

/************************************************************************/
CheckCanBus::CheckCanBus() {}



/************************************************************************/
// -- Función principal: Se llama en mod.runModule(rf) desde el main.cpp
bool CheckCanBus::configure(yarp::os::ResourceFinder &rf)
{

    timeOut = 2;        // -- Por defecto 2 [s] el parámetro --timeOut
    firstTime = 0;      // -- inicializo el tiempo a 0 [s] la primera vez que arranca el programa
    cleaningTime = 1;   // -- Por defecto 1 [s] el parámetro --cleaningTime
    bool flag;          // -- flag de return

    // -- Antes de configurar los periféricos, check a parámetro --help
    if(rf.check("help"))
    {
        std::printf("CheckCanBus options:\n");
        std::printf("\t--help (this help)\t --ids [\"(id)\"] \t\t --from [file.ini]\t --canDevice [path]*\n\t--timeOut [s]\t\t --resetAll (for all nodes)\t --resetNode [node]\t --cleaningTime [s]\n");
        std::printf("\n");
        std::printf("*can0: \t--canDevice /dev/can0\t\t can1: --canDevice /dev/can1\n");
        std::printf(" .ini:\t checkLocomotionCan0.ini\t checkLocomotionCan1.ini\t checkManipulationCan0.ini\t checkManipulationCan1.ini\n\n");
        std::printf("Example of uses:\n");
        std::printf("* Reset driver ID [23] and check it:\t\t\t\t checkCanBus --canDevice /dev/can1 --ids 23 --resetNode 23\n");
        std::printf("* Reset drivers IDs [23,24] and check them:\t\t\t checkCanBus --canDevice /dev/can1 --ids \"(23 24)\" --resetAll\n");
        std::printf("* Reset all drivers of manipulation Can0 and check devices:\t checkCanBus --canDevice /dev/can0 --from checkManipulationCan0.ini --resetAll\n");


        CD_DEBUG_NO_HEADER("%s\n",rf.toString().c_str());
        std::exit(1);
        return false;
    }

    // -- Continuación del código que CONFIGURA LA HICO-CAN y LOS DRIVERS
    CD_DEBUG("%s\n",rf.toString().c_str()); // -- nos muestra el contenido del objeto resource finder
    deviceDevCan0.open(rf);                 // -- Abre el dispositivo HicoCan (tarjeta) y le pasa el ResourceFinder
    if (!deviceDevCan0.isValid())
    {
        CD_ERROR("deviceDevCan0 instantiation not worked.\n");
        return false;
    }
    deviceDevCan0.view(iCanBus);            // -- conecta el dispositivo (hicocan)
    deviceDevCan0.view(iCanBufferFactory);

    canInputBuffer = iCanBufferFactory->createBuffer(1);

    // -- adding configuration of TechnosoftIpos (se trata de la configuración mínima que necesita el driver)
    yarp::os::Property TechnosoftIposConf("(device TechnosoftIpos) (canId 24) (min -100) (max 10) (tr 160) (refAcceleration 0.575) (refSpeed 5.0) (encoderPulses 1024)"); // -- frontal left elbow (codo)

    yarp::os::Value v(&iCanBufferFactory, sizeof(iCanBufferFactory));
    TechnosoftIposConf.put("canBufferFactory", v);

    bool ok = true;
    ok &= canNodeDevice.open( TechnosoftIposConf );   // -- we introduce the configuration properties defined ........
    ok &= canNodeDevice.view( iControlLimits2Raw );
    ok &= canNodeDevice.view( iControlModeRaw );
    ok &= canNodeDevice.view( iEncodersTimedRaw );
    ok &= canNodeDevice.view( iPositionControlRaw );
    ok &= canNodeDevice.view( iPositionDirectRaw );
    ok &= canNodeDevice.view( iTorqueControlRaw );
    ok &= canNodeDevice.view( iVelocityControlRaw );
    ok &= canNodeDevice.view( iCanBusSharer );
    ok &= canNodeDevice.view( technosoftIpos );   // -- conecta el dispositivo (drivers)

    // --Checking
    if(ok)
    {
        CD_SUCCESS("Configuration of TechnosoftIpos sucessfully :)\n");
    }
    else
    {
        CD_ERROR("Bad Configuration of TechnosoftIpos :(\n");
        std::exit(1);
    }

    //-- Pass CAN bus (HicoCAN) pointer to CAN node (TechnosoftIpos).
    iCanBusSharer->setCanBusPtr( iCanBus );

    flag = this->start(); // arranca el hilo

    // -- Parametro: --timeout [s]
    if(rf.check("timeOut"))
    {
        timeOut = rf.find("timeOut").asInt();
        CD_INFO_NO_HEADER("[INFO] Timeout: %.2f [s]\n", timeOut);
    }


    // -- Parametro: --resetAll
    if(rf.check("resetAll"))
    {
        CD_INFO_NO_HEADER("[INFO] Reseting all nodes\n");
        // -- doing reset of node after delay
        yarp::os::Time::delay(1);
        technosoftIpos->resetNodes();
    }

    // -- Parametro: --resetNode
    if(rf.check("resetNode"))
    {
        nodeForReset = rf.find("resetNode").asInt();
        CD_INFO_NO_HEADER("[INFO] Reseting node number: %i\n", nodeForReset );
        // -- doing reset of node after delay
        yarp::os::Time::delay(1);
        technosoftIpos->resetNode(nodeForReset);
    }


    // -- Parametro: --cleaningTime [s]
    if(rf.check("cleaningTime"))
    {
        cleaningTime = rf.find("cleaningTime").asInt();
        CD_INFO_NO_HEADER("[INFO] Cleaning Time: %.2f [s]\n", cleaningTime);
    }


    // -- Parametro: --ids (introduce los IDs en una cola)
    if(rf.check("ids"))
    {
        yarp::os::Bottle jointsCan0 = rf.findGroup("ids");  // -- Introduce en un objeto bottle el parámetro ids
        std::string strIds = jointsCan0.get(1).toString(); // -- strIds almacena los Ids que queremos comprobar
        std::stringstream streamIds(strIds); // --  tratamos el string de IDs como un stream llamado streamIds
        CD_INFO_NO_HEADER("[INFO] It will proceed to detect IDs: ");
        int n;
        while(streamIds>>n)     // -- recorre el stream y va introduciendo cada ID en la cola
        {
            std::printf("%i ",n);
            queueIds.push(n); // -- introduce en la cola los IDs
        }
        std::printf("\n");
    }


    /*  -- Parametro: --from (con este parámetro indicamos el lugar del .ini donde especificamos la configuración
     *  de la HicoCan y las ids que queremos comprobar)
     */
    else
    {
        yarp::os::Bottle fileIds = rf.findGroup("ids");
        std::string strIds = fileIds.get(1).toString(); // -- strIds almacena los Ids que queremos comprobar
        std::stringstream streamIds(strIds); // --  tratamos el string de IDs como un stream llamado streamIds
        CD_INFO_NO_HEADER("[INFO] It will proceed to detect IDs: ");
        int n;
        while(streamIds>>n)     // -- recorre el stream y va introduciendo cada ID en la cola
        {
            std::printf("%i ",n);
            queueIds.push(n); // -- introduce en la cola los IDs
        }
        std::printf("\n");
    }

    lastNow = yarp::os::Time::now(); // -- tiempo actual

    return flag;
}

/************************************************************************/
bool CheckCanBus::updateModule()
{
    //printf("CheckCanBus alive...\n");
    return true;
}

/************************************************************************/
// -- Para el hilo y para el dispositivo PolyDriver
bool CheckCanBus::close()
{
    this->stop();

    iCanBufferFactory->destroyBuffer(canInputBuffer);
    deviceDevCan0.close();

    return true;
}

/************************************************************************/
// -- Función que lee los mensajes que le llegan del CAN-BUS
std::string CheckCanBus::msgToStr(yarp::dev::CanMessage* message)
{

    std::stringstream tmp; // -- nos permite insertar cualquier tipo de dato dentro del flujo
    for(int i=0; i < message->getLen()-1; i++)
    {
        tmp << std::hex << static_cast<int>(message->getData()[i]) << " ";
    }
    tmp << std::hex << static_cast<int>(message->getData()[message->getLen()-1]);
    tmp << ". canId(";
    tmp << std::dec << (message->getId() & 0x7F); // -- muestra en decimal el ID
    tmp << ") via(";
    tmp << std::hex << (message->getId() & 0xFF80); // -- ???
    tmp << "), t:" << yarp::os::Time::now() - lastNow << "[s]."; // -- diferencia entre el tiempo actual y el del último mensaje

    lastNow = yarp::os::Time::now();    // -- tiempo actual (aleatorio)

    return tmp.str(); // -- devuelve el string (mensaje)
}

/*************************************************************************/

// -- Función que comprueba los mensajes que recibe del CAN utilizando una cola de IDs
void CheckCanBus::checkIds(yarp::dev::CanMessage* message)
{
    // -- Almacenamos el contenido del mensaje en un stream
    std::stringstream tmp; // -- stream que almacenará el mensaje recibido
    for(int i=0; i < message->getLen()-1; i++)
        tmp << std::hex << static_cast<int>(message->getData()[i]) << " ";   // -- inserta los bytes del mensaje menos el último
    tmp << std::hex << static_cast<int>(message->getData()[message->getLen()-1]); // -- inserta último byte

    // -- Recorremos la cola en busca del ID que ha lanzado el CAN
    for(int i=0; i<queueIds.size(); i++)   // -- bucle que recorrerá la cola
    {

        if(queueIds.front()== (message->getId() & 0x7F))     // -- si el ID coincide, lo saco de la cola
        {
            CD_SUCCESS_NO_HEADER("Detected ID: %i\n", queueIds.front());
            if(!tmp.str().compare("80 85 1 0 0 0 0 0")) CD_WARNING_NO_HEADER("[WARNING] Detected possible cleaning of driver settings: %i\n ", queueIds.front());
            queueIds.pop(); // -- saca de la cola el elemento
        }
        else                                            // -- En caso de que no coincida el ID
        {
            int res = queueIds.front(); // -- residuo que volveriamos a introducir en la cola
            queueIds.pop();      // -- saca de la cola el primer elemento
            queueIds.push(res);  // -- lo vuelve a introducir al final
        }
    }
}

// -- Función que imprime por pantalla los IDs no detectados (IDs residuales en cola)
void CheckCanBus::printWronglIds()
{
    for(int i=0; i<queueIds.size(); i++)
    {
        CD_ERROR_NO_HEADER("Has not been detected ID: %i\n", queueIds.front());
        queueIds.pop(); // -- saca de la cola el elemento
    }
}

/************************************************************************/

}  // namespace roboticslab
