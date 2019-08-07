// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraSerialControlboard.hpp"

using namespace roboticslab;

SerialSynapse::SerialSynapse(yarp::dev::ISerialDevice * _iSerialDevice)
    : iSerialDevice(_iSerialDevice)
{}

bool SerialSynapse::getMessage(unsigned char * msg, char stopByte, int size)
{
    char chr;
    while (!iSerialDevice->receiveChar(chr) || chr != stopByte) {}
    return iSerialDevice->receiveBytes(msg, size) != 0;
}

bool SerialSynapse::sendMessage(char * msg, int size)
{
    return iSerialDevice->send(msg, size);
}
