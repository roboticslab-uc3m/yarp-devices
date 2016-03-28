// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <ctime>
#include <cstdio>
#include "CheckCanBus.hpp"

// ------------------ Thread Related -----------------------------------------

void teo::CheckCanBus::run() {

    CD_INFO("Started CheckCanBus reading thread run.\n");


    while ( ! this->RFModule::isStopping() ) { // -- Mientras no se pare el RFModule



        struct can_msg buffer; // -- Mensaje CAN

        //-- read_timeout() returns the number read, -1 for errors or 0 for timeout or EOF.
        int ret = iCanBus->read_timeout(&buffer,1);

        //-- All debugging messages should be contained in read_timeout, so just loop again.
        if( ret <= 0 ) continue; // -- ?????

        int canId = buffer.id  & 0x7F; // -- limpia basura del CAN

        //-- Commenting next line as way too verbose, happens all the time.
        //CD_DEBUG("Read ok. %s\n", msgToStr(&buffer).c_str());

        //-- Intercept 700h 0 msg that just indicates presence.
        if( (buffer.id-canId) == 0x700 ) { // -- Filtra mensajes por presencia
            if(bootTime == 0) bootTime = yarp::os::Time::now(); // -- toma el bootTime al detectar la primera presencia

            //CD_SUCCESS("Device indicating presence. %s\n",msgToStr(&buffer).c_str());
            checkIds(&buffer);
            continue;
        }

        if(yarp::os::Time::now()-bootTime > 19) printf("Han transcurrido 20 segundos\n");
        printf("time: %i\n", yarp::os::Time::now()-bootTime);
        //CD_SUCCESS("Read CAN message: %s\n", msgToStr(&buffer).c_str()); // -- lee lo que le llega del can bus


    }  //-- ends: while ( ! this->isStopping() ).

    CD_INFO("Stopping DumpCanBus reading thread run.\n");

    return;
}

// -----------------------------------------------------------------------------

