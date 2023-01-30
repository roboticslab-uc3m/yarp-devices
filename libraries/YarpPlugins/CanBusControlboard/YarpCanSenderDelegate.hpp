// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __YARP_CAN_SENDER_DELEGATE_HPP__
#define __YARP_CAN_SENDER_DELEGATE_HPP__

#include <mutex>

#include <yarp/dev/CanBusInterface.h>

#include "ICanSenderDelegate.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusControlboard
 * @brief A sender delegate that adheres to standard YARP interfaces for CAN.
 */
class YarpCanSenderDelegate : public ICanSenderDelegate
{
public:
    //! Constructor, takes a reference to an existing CAN message buffer.
    YarpCanSenderDelegate(yarp::dev::CanBuffer & _buffer, std::mutex & _bufferMutex,
            unsigned int & n, unsigned int size)
        : buffer(_buffer),
          bufferMutex(_bufferMutex),
          preparedMessages(n),
          maxSize(size)
    {}

    bool prepareMessage(const can_message & msg) override;

private:
    yarp::dev::CanBuffer & buffer;
    std::mutex & bufferMutex;
    unsigned int & preparedMessages;
    unsigned int maxSize;
};

} // namespace roboticslab

#endif // __YARP_CAN_SENDER_DELEGATE_HPP__
