// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanSenderDelegate.hpp"

using namespace roboticslab;

CanSenderDelegate::CanSenderDelegate(yarp::dev::CanBuffer & _buffer, std::mutex & _bufferMutex)
    : buffer(_buffer),
      bufferMutex(_bufferMutex),
      preparedMessages(0)
{}

template <typename MessageConsumerT>
bool CanSenderDelegate::prepareMessage(MessageConsumerT callback)
{
    std::lock_guard<std::mutex> lock(bufferMutex);
    callback(buffer[preparedMessages++]);
    return true;
}

int CanSenderDelegate::getPreparedMessages() const
{
    return preparedMessages;
}

void CanSenderDelegate::resetPreparedMessages()
{
    preparedMessages = 0;
}
