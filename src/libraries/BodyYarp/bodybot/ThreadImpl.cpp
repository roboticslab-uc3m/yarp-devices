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
        CD_INFO("Read from fullCanId: %d (%d after mask)\n", buffer.id, canId);

        //idxFromCanId[canId]



    }  //-- ends: while ( ! this->isStopping() ).

    CD_INFO("Last thread run.\n");

    return;
}

// -----------------------------------------------------------------------------


