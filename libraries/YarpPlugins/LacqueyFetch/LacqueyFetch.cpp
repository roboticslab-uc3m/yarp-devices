// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <cstring>

// -----------------------------------------------------------------------------

std::string roboticslab::LacqueyFetch::msgToStr(yarp::dev::CanMessage * message)
{
    std::stringstream tmp;
    for(int i=0; i < message->getLen()-1; i++)
    {
        tmp << std::hex << static_cast<int>(message->getData()[i]) << " ";
    }
    tmp << std::hex << static_cast<int>(message->getData()[message->getLen()-1]);
    tmp << ". canId(";
    tmp << std::dec << (message->getId() & 0x7F);
    tmp << ") via(";
    tmp << std::hex << (message->getId() & 0xFF80);
    tmp << ").";
    return tmp.str();
}

// -----------------------------------------------------------------------------

std::string roboticslab::LacqueyFetch::msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData)
{
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

bool roboticslab::LacqueyFetch::send(uint32_t cob, uint16_t len, uint8_t * msgData)
{

    if ( (lastUsage - yarp::os::Time::now()) < DELAY )
        yarp::os::Time::delay( lastUsage + DELAY - yarp::os::Time::now() );

    yarp::dev::CanMessage &msg = canOutputBuffer[0];
    msg.setId(cob + canId);
    msg.setLen(len);
    std::memcpy(msg.getData(), msgData, len * sizeof(uint8_t));

    unsigned int sent;

    if( ! canDevicePtr->canWrite(canOutputBuffer, 1, &sent, true) || sent == 0 )
        return false;

    lastUsage = yarp::os::Time::now();
    return true;
}

// -----------------------------------------------------------------------------
