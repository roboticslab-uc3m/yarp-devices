// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// ------------------ RateThread Related -----------------------------------------

void teo::BodyBot::run() {

    while ( ! this->isStopping() ) {

        struct can_msg buffer;

        //-- read_timeout() returns the number read, -1 for errors or 0 for timeout or EOF.
        int ret = canDevice.read_timeout(&buffer,1);

        //-- All debugging messages should be contained in read_timeout, so just loop again.
        if( ret <= 0 ) continue;

        int canId = buffer.id  & 0x7F;

        //-- Commenting next line as way too verbose, happens all the time.
        //CD_DEBUG("Read from fullCanId: %d (%d after mask)\n", buffer.id, canId);

        std::map< int, int >::iterator idxFromCanIdFound = idxFromCanId.find(canId);

        if( idxFromCanIdFound == idxFromCanId.end() )
        {
            CD_ERROR("Read CAN message from unknown device!!! canId: %d.\n",canId);
            continue;
        }

        drivers[ idxFromCanIdFound->second ]->interpretMessage(&buffer);

    }  //-- ends: while ( ! this->isStopping() ).

    CD_INFO("Last thread run.\n");

    return;
}

// -----------------------------------------------------------------------------

