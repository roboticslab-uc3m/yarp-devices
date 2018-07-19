// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CheckCanBus.hpp"

#include <yarp/os/Time.h>

#include <ColorDebug.h>

// ------------------ Thread Related -----------------------------------------

void roboticslab::CheckCanBus::run()
{
    CD_INFO("Started CheckCanBus reading thread run.\n");

    double threadInitTime = yarp::os::Time::now();
    //double cleaningTime = 2.0;  // -- Convertir en argumento de programa

    while ( ! this->RFModule::isStopping())   // -- Mientras no se pare el RFModule
    {
        unsigned int read;

        //-- Blocks with timeout until one message arrives, returns false on errors.
        bool ok = iCanBus->canRead(canInputBuffer, 1, &read, true);

        /* Nota para la siguiente linea: si el tiempo que tarda el usuario en encender los brazos
         * es inferior al cleaningTime, puede dar lugar a pérdida de mensajes que no sean detectados por la
         * aplicación */
        if((yarp::os::Time::now()-threadInitTime) < cleaningTime) continue; //-- hasta que no llegue al cleaningTime, no revisará lo siguiente

        //-- All debugging messages should be contained in canRead, so just loop again.
        if( !ok || read == 0 ) continue; // --  continue para omitir secciones de código e iniciar la siguiente iteración de un bucle
        //  --  de esta forma se saltaría el código siguiente hasta que read > 0

        yarp::dev::CanMessage &msg = canInputBuffer[0];
        int canId = msg.getId() & 0x7F; // -- limpia basura del CAN

        //-- Intercept 700h 0 msg that just indicates presence.
        if( (msg.getId()-canId) == 0x700 )   // -- Filtra mensajes por presencia
        {
            if(firstTime == 0) firstTime = yarp::os::Time::now(); // -- toma el firsTime al detectar la primera presencia

            //CD_SUCCESS("Device indicating presence. %s\n",msgToStr(&buffer).c_str());
            checkIds(&msg); // -- Comprueba los IDs e imprime por pantalla los detectados
            continue; // -- Mientras esté detectando presencia, las instrucciones de abajo no las ejecuta
        }

        //----------------- Comprueba IDs de encoders (mensajes con otra cabecera) -----------------
        else                               // -- En caso de que NO sean mensajes de presencia
        {
            checkIds(&msg); // -- muestra en pantalla los IDs de los encoders detectados
            // -- Transcurridos los segundos indicados, imprime por pantalla los IDs no detectados
            if(int(yarp::os::Time::now()-firstTime)==timeOut+1)
                printWronglIds(); // -- Imprime los IDs que no se han utilizado
            // -- 2 segundos después, para el Modulo
            if(int(yarp::os::Time::now()-firstTime)==timeOut+3)
            {
                CD_SUCCESS_NO_HEADER("Happy end :)\n");
                this->stopModule();
            }
        }
        //CD_SUCCESS_NO_HEADER("Read CAN message: %s\n", msgToStr(&buffer).c_str()); // -- lee lo que le llega del can bus


    }  //-- ends: while ( ! this->isStopping() ).

    CD_INFO("Stopping CheckCanBus reading thread run.\n");

    return;
}

// -----------------------------------------------------------------------------

