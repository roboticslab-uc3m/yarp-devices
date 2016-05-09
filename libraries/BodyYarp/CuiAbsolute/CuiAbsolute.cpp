// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

// -----------------------------------------------------------------------------
// -- Lee un mensaje que proviene del CanBus (misma función utilizada en dumpCanBus y checkCanBus)
std::string teo::CuiAbsolute::msgToStr(can_msg* message) {
    std::stringstream tmp;
    for(int i=0; i < message->dlc-1; i++)
    {
        tmp << std::hex << static_cast<int>(message->data[i]) << " ";
    }
    tmp << std::hex << static_cast<int>(message->data[message->dlc-1]);
    tmp << ". canId(";
    tmp << std::dec << (message->id & 0x7F);
    tmp << ") via(";
    tmp << std::hex << (message->id & 0xFF80);
    tmp << ").";
    return tmp.str();
}

// -- Función del programa CheckCaBus para leer los mensajes que le llegan del CAN-BUS
/************************************************************************/
// -- Función que lee los mensajes que le llegan del CAN-BUS
// -- Ejemplo de lo que imprime la función en pantalla:
//    Read CAN message: 42 d5 b3 43. canId(126) via(180), t:0.0178779[s]
/*
std::string CheckCanBus::msgToStr(can_msg* message) {

    std::stringstream tmp; // -- nos permite insertar cualquier tipo de dato dentro del flujo
    for(int i=0; i < message->dlc-1; i++) // -- recorre en un bucle el contenido del mensaje (en bytes)
    {
        tmp << std::hex << static_cast<int>(message->data[i]) << " "; //-- cada byte es un número hexadecimal que compone el mensaje
    }
    tmp << std::hex << static_cast<int>(message->data[message->dlc-1]); //-- introduce el último byte (número) en hexadecimal
    tmp << ". canId(";
    tmp << std::dec << (message->id & 0x7F); // -- muestra en decimal el ID
    tmp << ") via(";
    tmp << std::hex << (message->id & 0xFF80); // -- ???
    tmp << "), t:" << yarp::os::Time::now() - lastNow << "[s]."; // -- diferencia entre el tiempo actual y el del último mensaje

    lastNow = yarp::os::Time::now();    // -- tiempo actual (aleatorio)

    return tmp.str(); // -- devuelve el string (mensaje)
}
*/

// -----------------------------------------------------------------------------
// -- Función que lee ... ¿?¿?¿?¿?¿?
/*
 * Write message to the CAN buffer.
 * @param cob Message's COB ???????????????
 * @param len Data field length
 * @param msgData Data
 * @return the message
*/
std::string teo::CuiAbsolute::msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData) {
    std::stringstream tmp; // -- nos permite insertar cualquier tipo de dato dentro del flujo
    for(int i=0; i < len-1; i++)
    {
        tmp << std::hex << static_cast<int>(*(msgData+i)) << " "; // -- nos permite acceder
    }
    tmp << std::hex << static_cast<int>(*(msgData+len-1));
    tmp << ". canId(";
    tmp << std::dec << canId;
    tmp << ") via(";
    tmp << std::hex << cob;
    tmp << ").";
    return tmp.str();
}

// -----------------------------------------------------------------------------
/*
 * Write message to the CAN buffer.
 * @param cob Message's COB
 * @param len Data field length
 * @param msgData Data to send
 * @return true/false on success/failure.
*/
bool teo::CuiAbsolute::sendDatatoPic(uint32_t cob, uint16_t len, uint8_t *msgData){

    if ( (lastUsage - yarp::os::Time::now()) < DELAY )
        yarp::os::Time::delay( lastUsage + DELAY - yarp::os::Time::now() );

    if( ! canDevicePtr->sendRaw(cob + this->canId, len, msgData) )
        return false;

    lastUsage = yarp::os::Time::now();
    return true;
}

// -----------------------------------------------------------------------------

/*
 * Write message to the CAN buffer.
 * @param cob Message's COB
 * @param len Data field length
 * @param msgData Data to send
 * @return true/false on success/failure.
*/
bool teo::CuiAbsolute::send(uint32_t cob, uint16_t len, uint8_t * msgData) {

    if ( (lastUsage - yarp::os::Time::now()) < DELAY )
        yarp::os::Time::delay( lastUsage + DELAY - yarp::os::Time::now() );

    if( ! canDevicePtr->sendRaw(cob + this->canId, len, msgData) )
        return false;

    lastUsage = yarp::os::Time::now();
    return true;
}

