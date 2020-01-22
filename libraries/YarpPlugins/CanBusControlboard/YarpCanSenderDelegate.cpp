// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "YarpCanSenderDelegate.hpp"

#include <cstring>

using namespace roboticslab;

bool YarpCanSenderDelegate::prepareMessage(const can_message & msg)
{
    std::lock_guard<std::mutex> lock(bufferMutex);

    if (preparedMessages < maxSize)
    {
        yarp::dev::CanMessage & message = buffer[preparedMessages++]; // access, then increment

        message.setId(msg.id);
        message.setLen(msg.len);

        if (msg.data)
        {
            std::memcpy(message.getData(), msg.data, msg.len);
        }

        return true;
    }

    return false;
}
