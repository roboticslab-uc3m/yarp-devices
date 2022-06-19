// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SocketCanMessage.hpp"

#include <cstring> // memcpy

using namespace roboticslab;

// -----------------------------------------------------------------------------

yarp::dev::CanMessage & SocketCanMessage::operator=(const yarp::dev::CanMessage & l)
{
    const auto & tmp = dynamic_cast<const SocketCanMessage &>(l);
    std::memcpy(message, tmp.message, sizeof(struct can_frame));
    return *this;
}

// -----------------------------------------------------------------------------

unsigned int SocketCanMessage::getId() const
{
    return message->can_id & CAN_SFF_MASK;
}

// -----------------------------------------------------------------------------

unsigned char SocketCanMessage::getLen() const
{
    return message->can_dlc;
}

// -----------------------------------------------------------------------------

void SocketCanMessage::setLen(unsigned char len)
{
    message->can_dlc = len;
}

// -----------------------------------------------------------------------------

void SocketCanMessage::setId(unsigned int id)
{
    message->can_id = id;
}

// -----------------------------------------------------------------------------

const unsigned char * SocketCanMessage::getData() const
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * SocketCanMessage::getData()
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * SocketCanMessage::getPointer()
{
    return reinterpret_cast<unsigned char *>(message);
}

// -----------------------------------------------------------------------------

const unsigned char * SocketCanMessage::getPointer() const
{
    return reinterpret_cast<const unsigned char *>(message);
}

// -----------------------------------------------------------------------------

void SocketCanMessage::setBuffer(unsigned char * buf)
{
    if (buf)
    {
        message = reinterpret_cast<struct can_frame *>(buf);
    }
}

// -----------------------------------------------------------------------------
