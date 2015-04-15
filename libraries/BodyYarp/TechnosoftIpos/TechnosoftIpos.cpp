// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// -----------------------------------------------------------------------------

std::string teo::TechnosoftIpos::msgToStr(can_msg* message) {
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

std::string teo::TechnosoftIpos::msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData) {
    std::stringstream tmp;
    for(int i=0; i < len-1; i++)
    {
        tmp << std::hex << static_cast<int>(*(msgData+i)) << " ";
    }
    tmp << std::hex << static_cast<int>(*(msgData+len-1));
    tmp << ". canId(";
    tmp << std::dec << canId;
    tmp << ") via(";
    tmp << std::hex << cob;
    tmp << ").";
    return tmp.str();
}

// -----------------------------------------------------------------------------

bool teo::TechnosoftIpos::send(uint32_t cob, uint16_t len, uint8_t * msgData) {

    if ( (lastUsage - yarp::os::Time::now()) < DELAY )
        yarp::os::Time::delay( lastUsage + DELAY - yarp::os::Time::now() );

    if( ! canDevicePtr->sendRaw(cob + this->canId, len, msgData) )
        return false;

    lastUsage = yarp::os::Time::now();
    return true;
}

// -----------------------------------------------------------------------------
