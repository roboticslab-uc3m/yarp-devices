// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BodyBot.hpp"

// -----------------------------------------------------------------------------

bool teo::BodyBot::indexWithinRange(const int& idx) {
    if (idx >= drivers.size() ){
        CD_WARNING("Index out of range!! (%d >= %zd)!!!\n",idx,drivers.size());
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::checkStatus(const int &j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    //*************************************************************
    uint8_t msgStatus[] = {0x40,0x41,0x60,0x00,0x00,0x00,0x00,0x00}; // Manual 6041h: Status word
    if( ! drivers[j]->send( 0x600, 8, msgStatus))
    {
        CD_ERROR("Could not send status query.\n");
        return false;
    }
    CD_SUCCESS("Sent status query to canId: %d.\n",drivers[j]->getCanId());
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

void teo::BodyBot::display(struct can_msg message) {
    int i;
    printf(" [ ID: %X | ",message.id);
    printf("Data: ");
    for(i=0; i < message.dlc; i++){
        printf("%X - ",message.data[i]);
    }
    printf("]\n");
}

// -----------------------------------------------------------------------------

bool teo::BodyBot::waitSequence(int len) {
    CD_INFO("\n");

    int n=0;
    struct can_msg reply;
    int waitt=0;

    while(n<len)
    {
        if(canDevice.read_timeout(&reply,20))
        {
            display(reply);
            waitt=0;
            if((reply.id>0x180) && (reply.id<0x280) && (reply.data[1] & 4)) //mask 0000 0100 0000 0000 status word - Target reached
            {
                CD_INFO("Move %d finished.\n",n);
                n++;
            }
        }
        else
        {
            waitt+=50;
            if (waitt > 5000)
            {
                CD_WARNING("Wait timeout.\n");
                return false;
            }
        }
    }

    return true;
}

// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
