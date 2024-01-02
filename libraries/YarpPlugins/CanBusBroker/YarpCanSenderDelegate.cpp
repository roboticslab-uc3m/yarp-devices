// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "YarpCanSenderDelegate.hpp"

#include <cstring>

#include <algorithm> // std::any_of

using namespace roboticslab;

bool YarpCanSenderDelegate::prepareMessage(const can_message & msg)
{
    if (!isActive)
    {
        return false;
    }

    std::lock_guard lock(bufferMutex);

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

void YarpCanSenderDelegate::reportAvailability(bool available, unsigned int id)
{
    // it is assumed that this method won't be called concurrently (check TechnosoftIpos::monitorWorker),
    // hence no mutex is required here
    nodeAvailability[id] = available;

    isActive = std::any_of(nodeAvailability.cbegin(), nodeAvailability.cend(),
                           [](const auto & pair) { return pair.second; });
}
