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
    uint8_t msgStatus[] = {0x40,0x41,0x60,0x00,0x00,0x00,0x00,0x00}; //2064: Memory position
    if( ! drivers[j]->send( 0x600, 8, msgStatus))
    {
        CD_ERROR("Could not send status query.\n");
        return false;
    }
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*
    bool received=false;
    can_msg buffer;

    do
    {
        if( canDevice.read_timeout(&buffer,20))
        {
            received=((buffer.data[1]== 0x41) && (buffer.data[2]=0x60));
        } else {
            CD_WARNING("Read timeout!\n");
            return false;
        }
    }
    while(!received);*/
    //*************************************************************

    // Big Endian
    /*CD_INFO("Drive of canId %d current status:\n", drivers[j]->getCanId());
    if(buffer.data[4] & 1){//0000 0001
        CD_INFO("\t-Ready to switch on.\n");
    }
    if(buffer.data[4] & 2){//0000 0010
        CD_INFO("\t-Switched on.\n");
    }
    if(buffer.data[4] & 4){//0000 0100
        CD_INFO("\t-Operation Enabled.\n");
    }
    if(buffer.data[4] & 8){//0000 0100
        CD_INFO("\t-In fault condition.\n");
    }
    if(buffer.data[4] & 16){//0000 1000
        CD_INFO("\t-Supply voltage absent.\n");
    }
    if(!(buffer.data[4] & 32)){//5th bit=0
        CD_INFO("\t-Performing a quick stop.\n");
    }
    if(buffer.data[4] & 64){//6th bit
        CD_INFO("\t-Switch on disabled.\n");
    }
    if(buffer.data[4] & 128){//7th bit
        CD_INFO("\t-Warning, a TML function is already in execution.\n");
    }
    if(buffer.data[5] & 1){//9th bit
        CD_INFO("\t-TML running.\n");
    }
    if(buffer.data[5] & 2){//9th bit
        CD_INFO("\t-Remote operation via CAN available.\n");
    }else{
        CD_INFO("\t-Only internal TML operation available.\n");
    }
    if(buffer.data[5] & 4){//9th bit
        CD_INFO("\t-Target reached.\n");
    }
    if(buffer.data[5] & 8){//9th bit
        CD_INFO("\t-Internal Limit Active.\n");
    }
    if(buffer.data[5] & 64){//9th bit
        CD_INFO("\t-Last event set has ocurred.\n");
    }
    if(buffer.data[5] & 128){//9th bit
        CD_INFO("\t-Axis on.\n");
    }*/

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
