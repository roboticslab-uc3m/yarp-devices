// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// ------------------ Thread Related -----------------------------------------

void teo::CanBusControlboard::run()
{

    CD_INFO("Started CanBusControlboard reading thread run.\n");

    while ( ! this->isStopping() )
    {

        struct can_msg buffer;

        //-- read_timeout() returns the number read, -1 for errors or 0 for timeout or EOF.
        int ret = iCanBus->read_timeout(&buffer,1);

        //-- All debugging messages should be contained in read_timeout, so just loop again.
        if( ret <= 0 ) continue;

        int canId = buffer.id  & 0x7F;

        //-- Commenting next line as way too verbose, happens all the time.
        //CD_DEBUG("Read ok. %s\n", msgToStr(&buffer).c_str());

        std::map< int, int >::iterator idxFromCanIdFound = idxFromCanId.find(canId);

        if( idxFromCanIdFound == idxFromCanId.end() )  //-- Can ID not found
        {
            //-- Intercept 700h 0 msg that just indicates presence.
            if( (buffer.id-canId) == 0x700 )
            {
                CD_SUCCESS("Device indicating presence. %s\n",msgToStr(&buffer).c_str());
                continue;
            }

            CD_ERROR("Read CAN message from unknown device!!! %s\n", msgToStr(&buffer).c_str());
            continue;
        }

        //CD_DEBUG("idxFromCanIdFound->second: %d\n",idxFromCanIdFound->second);
        iCanBusSharer[ idxFromCanIdFound->second ]->interpretMessage(&buffer);  //-- Check if false?

    }  //-- ends: while ( ! this->isStopping() ).

    CD_INFO("Stopping CanBusControlboard reading thread run.\n");

    return;
}

// -----------------------------------------------------------------------------

