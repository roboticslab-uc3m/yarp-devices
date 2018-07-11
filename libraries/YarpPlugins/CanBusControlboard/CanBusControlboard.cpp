// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::indexWithinRange(const int& idx)
{
    if (idx >= nodes.size() )
    {
        CD_WARNING("Index out of range!! (%d >= %zd)!!!\n",idx,nodes.size());
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------

std::string roboticslab::CanBusControlboard::msgToStr(const yarp::dev::CanMessage& message)
{

    std::stringstream tmp;
    for(int i=0; i < message.getLen()-1; i++)
    {
        tmp << std::hex << static_cast<int>(message.getData()[i]) << " ";
    }
    tmp << std::hex << static_cast<int>(message.getData()[message.getLen()-1]);
    tmp << ". canId(";
    tmp << std::dec << (message.getId() & 0x7F);
    tmp << ") via(";
    tmp << std::hex << (message.getId() & 0xFF80);
    tmp << ").";
    return tmp.str();
}

// -----------------------------------------------------------------------------
