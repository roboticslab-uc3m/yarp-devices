// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <SocketCanMessage.hpp>

#include <cstring>  // memcpy

// -----------------------------------------------------------------------------

roboticslab::SocketCanMessage::SocketCanMessage()
{
    message = 0;
}

// -----------------------------------------------------------------------------

roboticslab::SocketCanMessage::~SocketCanMessage()
{
}

// -----------------------------------------------------------------------------

yarp::dev::CanMessage & roboticslab::SocketCanMessage::operator=(const yarp::dev::CanMessage & l)
{
    const SocketCanMessage & tmp = dynamic_cast<const SocketCanMessage &>(l);
    std::memcpy(message, tmp.message, sizeof(struct can_frame));
    return *this;
}

// -----------------------------------------------------------------------------

unsigned int roboticslab::SocketCanMessage::getId() const
{
    return message->can_id & 0x1FFFFFFF;
}

// -----------------------------------------------------------------------------

unsigned char roboticslab::SocketCanMessage::getLen() const
{
    return message->can_dlc;
}

// -----------------------------------------------------------------------------

void roboticslab::SocketCanMessage::setLen(unsigned char len)
{
    message->can_dlc = len;
}

// -----------------------------------------------------------------------------

void roboticslab::SocketCanMessage::setId(unsigned int id)
{
    message->can_dlc &= 0xE0000000;
    message->can_id |= id;
}

// -----------------------------------------------------------------------------

const unsigned char * roboticslab::SocketCanMessage::getData() const
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * roboticslab::SocketCanMessage::getData()
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * roboticslab::SocketCanMessage::getPointer()
{
    return (unsigned char *)message;
}

// -----------------------------------------------------------------------------

const unsigned char * roboticslab::SocketCanMessage::getPointer() const
{
    return (const unsigned char *)message;
}

// -----------------------------------------------------------------------------

void roboticslab::SocketCanMessage::setBuffer(unsigned char * buf)
{
    if (buf != 0)
    {
        message = (struct can_frame *)buf;
    }
}

// -----------------------------------------------------------------------------
