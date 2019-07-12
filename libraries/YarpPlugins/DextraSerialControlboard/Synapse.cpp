// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <cstring>

#include <ColorDebug.h>
#include <DextraSerialControlboard/Synapse.hpp>

using namespace roboticslab;

namespace
{
    const char HEADER = 0x7E;
    const char FOOTER = 0x7E;
    const int FLOAT_SIZE = sizeof(Synapse::setpoint_t);
    const int MESSAGE_SIZE = (FLOAT_SIZE + 1) * Synapse::DATA_POINTS;
    const char FINGER_ADDRESS[Synapse::DATA_POINTS] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
}

const std::pair<Synapse::setpoint_t, Synapse::setpoint_t> Synapse::LIMITS[Synapse::DATA_POINTS] = {
    std::make_pair(0, 90),
    std::make_pair(0, 10),
    std::make_pair(0, 20),
    std::make_pair(0, 20),
    std::make_pair(0, 20),
    std::make_pair(0, 20)
};

const char * Synapse::LABELS[Synapse::DATA_POINTS] = {
    "abductor",
    "thumb",
    "index",
    "middle",
    "ring",
    "pinky"
};

Synapse::Synapse(yarp::dev::ISerialDevice * _iSerialDevice)
    : iSerialDevice(_iSerialDevice)
{}

bool Synapse::getMessage(unsigned char * msg)
{
    char chr;
    while (!iSerialDevice->receiveChar(chr) || chr != HEADER) {}
    return iSerialDevice->receiveBytes(msg, MESSAGE_SIZE + 2) != 0;
}

bool Synapse::readDataList(Setpoints & setpoints)
{
    unsigned char msg[MESSAGE_SIZE + 2]; // data (MESSAGE_SIZE) + check (1) + footer (1)

    if (!getMessage(msg))
    {
        return false;
    }

    int i = 0;

    for (int j = 0; j < DATA_POINTS; j++)
    {
        if (msg[i] == FINGER_ADDRESS[j])
        {
            i++;
            double setpoint;
            std::memcpy(&setpoint, msg + i, FLOAT_SIZE);
            i += FLOAT_SIZE;
            setpoints[j] = setpoint;
        }
    }

    return true;
}

bool Synapse::writeSetpointList(const Setpoints & setpoints)
{
    char check = 0x00;
    char msg[MESSAGE_SIZE + 3]; // header (1) + data (MESSAGE_SIZE) + check (1) + footer (1)

    msg[0] = HEADER;

    for (int i = 0; i < DATA_POINTS; i++)
    {
        const int offset = 1 + (FLOAT_SIZE + 1) * i;
        msg[offset] = FINGER_ADDRESS[i];
        std::memcpy(msg + offset + 1, &setpoints[i], FLOAT_SIZE);
        check ^= FINGER_ADDRESS[i];

        for (int k = 0; k < FLOAT_SIZE; k++)
        {
            check ^= msg[offset + 1 + k];
        }
    }

    msg[MESSAGE_SIZE + 3 - 2] = check;
    msg[MESSAGE_SIZE + 3 - 1] = FOOTER;

    return iSerialDevice->send(msg, MESSAGE_SIZE + 3);
}
