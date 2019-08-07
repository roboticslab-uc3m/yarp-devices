// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraCanControlboard.hpp"

using namespace roboticslab;

CanSynapse::CanSynapse()
{}

bool CanSynapse::getMessage(unsigned char * msg, char stopByte, int size)
{
    return true;
}

bool CanSynapse::sendMessage(char * msg, int size)
{
    return true;
}
