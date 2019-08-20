// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_SENDER_DELEGATE_HPP__
#define __CAN_SENDER_DELEGATE_HPP__

#include <mutex>

#include <yarp/dev/CanBusInterface.h>

namespace roboticslab
{

class message_builder
{
public:
    message_builder(int _id, int _len, const unsigned char * _data) : id(_id), len(_len), data(_data)
    {}

    void operator()(yarp::dev::CanMessage & msg)
    {
        msg.setId(id);
        msg.setLen(len);
        std::memcpy(msg.getData(), data, len * sizeof(data));
    }

private:
    int id;
    int len;
    const unsigned char * data;
};

/**
 * @brief
 */
class CanSenderDelegate
{
public:
    CanSenderDelegate(yarp::dev::CanBuffer & _buffer, std::mutex & _bufferMutex, int & n, int size)
        : buffer(_buffer),
          bufferMutex(_bufferMutex),
          preparedMessages(n),
          maxSize(size)
    {}

    template <typename MessageConsumerT>
    bool prepareMessage(MessageConsumerT callback)
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        return preparedMessages < maxSize ? callback(buffer[preparedMessages++]), true : false;
    }

private:
    yarp::dev::CanBuffer & buffer;
    std::mutex & bufferMutex;
    int & preparedMessages;
    int maxSize;
};

}  // namespace roboticslab

#endif // __CAN_SENDER_DELEGATE_HPP__
