// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SOCKET_CAN_MESSAGE__
#define __SOCKET_CAN_MESSAGE__

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
    SocketCanMessage();
    virtual ~SocketCanMessage();
    virtual yarp::dev::CanMessage & operator=(const yarp::dev::CanMessage & l);

    virtual unsigned int getId() const;
    virtual unsigned char getLen() const;
    virtual void setLen(unsigned char len);
    virtual void setId(unsigned int id);
    virtual const unsigned char * getData() const;
    virtual unsigned char * getData();
    virtual unsigned char * getPointer();
    virtual const unsigned char * getPointer() const;
    virtual void setBuffer(unsigned char * buf);

private:
};

}  // namespace roboticslab

#endif  // __SOCKET_CAN_MESSAGE__
