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

std::string teo::BodyBot::msgToStr(can_msg* message) {

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

// -----------------------------------------------------------------------------

/*bool teo::BodyBot::checkStatus(const int &j) {
    CD_INFO("(%d)\n",j);

    //-- Check index within range
    if ( ! this->indexWithinRange(j) ) return false;

    //*************************************************************
    uint8_t msgStatus[] = {0x40,0x41,0x60,0x00,0x00,0x00,0x00,0x00}; // Manual 6041h: Status word
    if( ! drivers[j]->send( 0x600, 8, msgStatus))
    {
        CD_ERROR("Could not send status query. %s.\n", msgToStr(0x600, 8, msgStatus).c_str() );
        return false;
    }
    CD_SUCCESS("Sent status query. %s.\n", msgToStr(0x600, 8, msgStatus).c_str() );
    //*************************************************************

    return true;
}*/

// -----------------------------------------------------------------------------
