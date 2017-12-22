// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DumpCanBus.hpp"

#include <ColorDebug.hpp>

// ------------------ Thread Related -----------------------------------------

void roboticslab::DumpCanBus::run()
{
    CD_INFO("Started DumpCanBus reading thread run.\n");

    while ( ! this->RFModule::isStopping() )
    {

        yarp::dev::CanMessage &msg = canInputBuffer[0];

        unsigned int read;

        //-- returns false for errors, timeout or EOF.
        bool ok = iCanBus->canRead(canInputBuffer, 1, &read, true);

        //-- All debugging messages should be contained in read_timeout, so just loop again.
        if( !ok ) continue;

        int canId = msg.getId()  & 0x7F;

        //-- Commenting next line as way too verbose, happens all the time.
        //CD_DEBUG("Read ok. %s\n", msgToStr(&buffer).c_str());

        //-- Intercept 700h 0 msg that just indicates presence.
        if( (msg.getId()-canId) == 0x700 )
        {
            CD_SUCCESS("Device indicating presence. %s\n",msgToStr(&msg).c_str());
            continue;
        }

        CD_SUCCESS("Read CAN message: %s\n", msgToStr(&msg).c_str());

    }  //-- ends: while ( ! this->isStopping() ).

    CD_INFO("Stopping DumpCanBus reading thread run.\n");

    return;
}

// -----------------------------------------------------------------------------
