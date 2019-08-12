// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraCanControlboard.hpp"

#include <cassert>
#include <cmath>
#include <cstring>

using namespace roboticslab;

#define BUFFER_SIZE 10

CanSynapse::CanSynapse(int _canId)
    : canId(_canId),
      sender(0)
{}

void CanSynapse::configure(void * handle)
{
    sender = static_cast<CanSenderDelegate *>(handle);
    configured = true;
}

bool CanSynapse::getMessage(unsigned char * msg, char stopByte, int size)
{
    return false;
}

bool CanSynapse::sendMessage(unsigned char * msg, int size)
{
    int n = std::ceil(size / 8.0f);
    assert(BUFFER_SIZE >= n);

    for (int i = 0; i < n; i++)
    {
        const int bytes = ((i + 1) * 8 <= size) ? 8 : size % 8;

        if (!sender->prepareMessage(message_builder(canId, bytes, msg)))
        {
            return false;
        }
    }

    return true;
}
