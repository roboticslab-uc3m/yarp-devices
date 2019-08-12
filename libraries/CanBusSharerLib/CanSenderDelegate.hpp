// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_SENDER_DELEGATE_HPP__
#define __CAN_SENDER_DELEGATE_HPP__

#include <mutex>

#include <yarp/dev/CanBusInterface.h>

namespace roboticslab
{

/**
 * @brief
 */
class CanSenderDelegate
{
public:
    typedef bool (*MessageFun)(yarp::dev::CanMessage &);

    CanSenderDelegate(yarp::dev::CanBuffer & buffer, std::mutex & bufferMutex);

    bool prepareMessage(MessageFun callback);
    int getPreparedMessages() const;
    void resetPreparedMessages();

private:
    yarp::dev::CanBuffer & buffer;
    std::mutex & bufferMutex;
    int preparedMessages;
};

}  // namespace roboticslab

#endif // __CAN_SENDER_DELEGATE_HPP__
