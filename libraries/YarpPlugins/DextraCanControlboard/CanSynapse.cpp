// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraCanControlboard.hpp"

#include <cassert>
#include <cmath>
#include <cstring>

using namespace roboticslab;

#define BUFFER_SIZE 10

CanSynapse::CanSynapse(int _canId, yarp::dev::ICanBus * _iCanBus, yarp::dev::ICanBufferFactory * _iCanBufferFactory)
    : canId(_canId),
      iCanBus(_iCanBus),
      iCanBufferFactory(_iCanBufferFactory),
      canBuffer(iCanBufferFactory->createBuffer(BUFFER_SIZE))
{}

CanSynapse::~CanSynapse()
{
    iCanBufferFactory->destroyBuffer(canBuffer);
}

bool CanSynapse::getMessage(unsigned char * msg, char stopByte, int size)
{
    return false;
}

bool CanSynapse::sendMessage(char * msg, int size)
{
    int n = std::ceil(size / 8.0f);
    assert(BUFFER_SIZE >= n);

    std::lock_guard<std::mutex> guard(mtx);

    for (int i = 0; i < n; i++)
    {
        const int bytes = ((i + 1) * 8 <= size) ? 8 : size % 8;
        yarp::dev::CanMessage & canMsg = canBuffer[i];
        canMsg.setId(canId);
        canMsg.setLen(bytes);
        std::memcpy(canMsg.getData(), msg, bytes);
    }

    unsigned int sent;
    return iCanBus->canWrite(canBuffer, n, &sent, true) && sent != 0;
}
