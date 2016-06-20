// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

// -----------------------------------------------------------------------------

bool teo::CanBusControlboard::indexWithinRange(const int& idx)
{
    if (idx >= nodes.size() )
    {
        CD_WARNING("Index out of range!! (%d >= %zd)!!!\n",idx,nodes.size());
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------

std::string teo::CanBusControlboard::msgToStr(can_msg* message)
{

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
