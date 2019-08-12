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
    message_builder(int _id, int _len, unsigned char * _data) : id(_id), len(_len), data(_data)
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
    unsigned char * data;
};

/**
 * @brief
 */
class CanSenderDelegate
{
public:
    CanSenderDelegate(yarp::dev::CanBuffer & buffer, std::mutex & bufferMutex);

    template <typename MessageConsumerT>
    bool prepareMessage(MessageConsumerT callback);

    int getPreparedMessages() const;
    void resetPreparedMessages();

private:
    yarp::dev::CanBuffer & buffer;
    std::mutex & bufferMutex;
    int preparedMessages;
};

}  // namespace roboticslab

#endif // __CAN_SENDER_DELEGATE_HPP__
