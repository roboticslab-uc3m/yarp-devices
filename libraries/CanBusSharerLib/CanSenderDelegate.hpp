// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_SENDER_DELEGATE_HPP__
#define __CAN_SENDER_DELEGATE_HPP__

#include <mutex>

#include <yarp/dev/CanBusInterface.h>

namespace roboticslab
{

struct message_builder
{
    void operator ()(yarp::dev::CanMessage & msg) const;

    unsigned int id;
    unsigned int len;
    const unsigned char * data;
};

/**
 * @brief
 */
class CanSenderDelegate
{
public:
    CanSenderDelegate(yarp::dev::CanBuffer & _buffer, std::mutex & _bufferMutex, unsigned int & n, unsigned int size)
        : buffer(_buffer),
          bufferMutex(_bufferMutex),
          preparedMessages(n),
          maxSize(size)
    {}

    bool prepareMessage(const message_builder & callback) const
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        return preparedMessages < maxSize ? callback(buffer[preparedMessages++]), true : false;
    }

private:
    yarp::dev::CanBuffer & buffer;
    std::mutex & bufferMutex;
    unsigned int & preparedMessages;
    unsigned int maxSize;
};

}  // namespace roboticslab

#endif // __CAN_SENDER_DELEGATE_HPP__
