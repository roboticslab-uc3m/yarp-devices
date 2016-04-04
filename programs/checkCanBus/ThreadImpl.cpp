// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <cstdio>
#include "CheckCanBus.hpp"

// ------------------ Thread Related -----------------------------------------

void teo::CheckCanBus::run() {


    CD_INFO("Started CheckCanBus reading thread run.\n");

    double threadInitTime = yarp::os::Time::now();
    double cleaningTime = 2.0;  // -- Convertir en argumento de programa

    while ( ! this->RFModule::isStopping()) { // -- Mientras no se pare el RFModule

        struct can_msg buffer; // -- Mensaje CAN

        //-- read_timeout() returns the number read, -1 for errors or 0 for timeout or EOF.
        int ret = iCanBus->read_timeout(&buffer,1);

        if((yarp::os::Time::now()-threadInitTime) < cleaningTime) continue;

        //-- All debugging messages should be contained in read_timeout, so just loop again.
        if( ret <= 0 ) continue; // --  continue para omitir secciones de código e iniciar la siguiente iteración de un bucle
                                //  --  de esta forma se saltaría el código siguiente hasta que (ret > 0 )

        int canId = buffer.id  & 0x7F; // -- limpia basura del CAN

        //-- Commenting next line as way too verbose, happens all the time.
        //CD_DEBUG("Read ok. %s\n", msgToStr(&buffer).c_str());

        //-- Intercept 700h 0 msg that just indicates presence.
        if( (buffer.id-canId) == 0x700 ) { // -- Filtra mensajes por presencia
            if(firstTime == 0) firstTime = yarp::os::Time::now(); // -- toma el firsTime al detectar la primera presencia

            //CD_SUCCESS("Device indicating presence. %s\n",msgToStr(&buffer).c_str());
            checkIds(&buffer); // -- Comprueba los IDs e imprime por pantalla los detectados
            continue; // -- Mientras esté detectando presencia, las instrucciones de abajo no las ejecuta
        }

        //----------------- Comprueba IDs de encoders (mensajes con otra cabecera) -----------------
        else {                             // -- En caso de que NO sean mensajes de presencia
            checkIds(&buffer); // -- muestra en pantalla los IDs de los encoders detectados
            // -- Transcurridos los segundos indicados, imprime por pantalla los IDs no detectados
            if(int(yarp::os::Time::now()-firstTime)==timeOut+1)
                printWronglIds(); // -- Imprime los IDs que no se han utilizado
            // -- 2 segundos después, para el Modulo
            if(int(yarp::os::Time::now()-firstTime)==timeOut+3) {
                printf("Happy end :)\n");
                this->stopModule();
            }
        }
        //CD_SUCCESS_NO_HEADER("Read CAN message: %s\n", msgToStr(&buffer).c_str()); // -- lee lo que le llega del can bus


    }  //-- ends: while ( ! this->isStopping() ).

    CD_INFO("Stopping CheckCanBus reading thread run.\n");

    return;
}

// -----------------------------------------------------------------------------

