// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Synapse.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

namespace
{
    const char HEADER = 0x7E;
    const char FOOTER = 0x7E;
    const int FLOAT_SIZE = 4;
    const int MESSAGE_SIZE = 30;
    const char FINGER_ADDRESS[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
}

Synapse::Synapse()
    : iSerialDevice(0)
{}

void Synapse::getMessage()
{}

bool Synapse::readDataList(std::vector<double> & setpoints)
{
    return true;
}

bool Synapse::writeSetpointList(const std::vector<double> & setpoints)
{
    return true;
}