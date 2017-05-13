// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HicoCanMessage.hpp"

#include <cstring>  // memcpy

// -----------------------------------------------------------------------------

roboticslab::HicoCanMessage::HicoCanMessage()
{
    message = 0;
}

// -----------------------------------------------------------------------------

roboticslab::HicoCanMessage::~HicoCanMessage()
{
}

// -----------------------------------------------------------------------------

yarp::dev::CanMessage & roboticslab::HicoCanMessage::operator=(const yarp::dev::CanMessage & l)
{
    const HicoCanMessage & tmp = dynamic_cast<const HicoCanMessage &>(l);
    std::memcpy(message, tmp.message, sizeof(can_msg));
    return *this;
}

// -----------------------------------------------------------------------------

unsigned int roboticslab::HicoCanMessage::getId() const
{
    return message->id;
}

// -----------------------------------------------------------------------------

unsigned char roboticslab::HicoCanMessage::getLen() const
{
    return message->dlc;
}

// -----------------------------------------------------------------------------

void roboticslab::HicoCanMessage::setLen(unsigned char len)
{
    message->dlc = len;
}

// -----------------------------------------------------------------------------

void roboticslab::HicoCanMessage::setId(unsigned int id)
{
    message->id = id;
}

// -----------------------------------------------------------------------------

const unsigned char * roboticslab::HicoCanMessage::getData() const
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * roboticslab::HicoCanMessage::getData()
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * roboticslab::HicoCanMessage::getPointer()
{
    return (unsigned char *) message;
}

// -----------------------------------------------------------------------------

const unsigned char * roboticslab::HicoCanMessage::getPointer() const
{
    return (const unsigned char *) message;
}

// -----------------------------------------------------------------------------

void roboticslab::HicoCanMessage::setBuffer(unsigned char * buf)
{
    if (buf != 0)
    {
        message = (can_msg *) buf;
    }
}

// -----------------------------------------------------------------------------
