// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SOCKET_CAN_MESSAGE__
#define __SOCKET_CAN_MESSAGE__

#include <linux/can.h>

#include <yarp/dev/CanBusInterface.h>

namespace roboticslab
{

/**
 * @ingroup CanBusSocket
 * @brief YARP wrapper for SocketCAN messages.
 */
class SocketCanMessage : public yarp::dev::CanMessage
{
public:
    SocketCanMessage() : message(nullptr)
    {}

    yarp::dev::CanMessage & operator=(const yarp::dev::CanMessage & l) override;

    unsigned int getId() const override;
    unsigned char getLen() const override;
    void setLen(unsigned char len) override;
    void setId(unsigned int id) override;
    const unsigned char * getData() const override;
    unsigned char * getData() override;
    unsigned char * getPointer() override;
    const unsigned char * getPointer() const override;
    void setBuffer(unsigned char * buf) override;

private:
    struct can_frame * message;
};

} // namespace roboticslab

#endif // __SOCKET_CAN_MESSAGE__
