// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HicoCanMessage.hpp"

#include <cstring>  // memcpy

using namespace roboticslab;

// -----------------------------------------------------------------------------

HicoCanMessage::HicoCanMessage()
{
    message = nullptr;
}

// -----------------------------------------------------------------------------

HicoCanMessage::~HicoCanMessage()
{
}

// -----------------------------------------------------------------------------

yarp::dev::CanMessage & HicoCanMessage::operator=(const yarp::dev::CanMessage & l)
{
    const HicoCanMessage & tmp = dynamic_cast<const HicoCanMessage &>(l);
    std::memcpy(message, tmp.message, sizeof(struct can_msg));
    return *this;
}

// -----------------------------------------------------------------------------

unsigned int HicoCanMessage::getId() const
{
    return message->id;
}

// -----------------------------------------------------------------------------

unsigned char HicoCanMessage::getLen() const
{
    return message->dlc;
}

// -----------------------------------------------------------------------------

void HicoCanMessage::setLen(unsigned char len)
{
    message->dlc = len;
}

// -----------------------------------------------------------------------------

void HicoCanMessage::setId(unsigned int id)
{
    message->id = id;
}

// -----------------------------------------------------------------------------

const unsigned char * HicoCanMessage::getData() const
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * HicoCanMessage::getData()
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * HicoCanMessage::getPointer()
{
    return reinterpret_cast<unsigned char *>(message);
}

// -----------------------------------------------------------------------------

const unsigned char * HicoCanMessage::getPointer() const
{
    return reinterpret_cast<const unsigned char *>(message);
}

// -----------------------------------------------------------------------------

void HicoCanMessage::setBuffer(unsigned char * buf)
{
    if (buf)
    {
        message = reinterpret_cast<struct can_msg *>(buf);
    }
}

// -----------------------------------------------------------------------------
