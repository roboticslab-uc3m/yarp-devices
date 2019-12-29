// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeCanMessage.hpp"

#include <cstring> // std::memcpy

// -----------------------------------------------------------------------------

roboticslab::FakeCanMessage::FakeCanMessage()
{
    message = nullptr;
}

// -----------------------------------------------------------------------------

roboticslab::FakeCanMessage::~FakeCanMessage()
{
}

// -----------------------------------------------------------------------------

yarp::dev::CanMessage & roboticslab::FakeCanMessage::operator=(const yarp::dev::CanMessage & l)
{
    const FakeCanMessage & tmp = dynamic_cast<const FakeCanMessage &>(l);
    std::memcpy(message, tmp.message, sizeof(struct fake_can_msg));
    return *this;
}

// -----------------------------------------------------------------------------

unsigned int roboticslab::FakeCanMessage::getId() const
{
    return message->id;
}

// -----------------------------------------------------------------------------

unsigned char roboticslab::FakeCanMessage::getLen() const
{
    return message->dlc;
}

// -----------------------------------------------------------------------------

void roboticslab::FakeCanMessage::setLen(unsigned char len)
{
    message->dlc = len;
}

// -----------------------------------------------------------------------------

void roboticslab::FakeCanMessage::setId(unsigned int id)
{
    message->id = id;
}

// -----------------------------------------------------------------------------

const unsigned char * roboticslab::FakeCanMessage::getData() const
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * roboticslab::FakeCanMessage::getData()
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * roboticslab::FakeCanMessage::getPointer()
{
    return reinterpret_cast<unsigned char *>(message);
}

// -----------------------------------------------------------------------------

const unsigned char * roboticslab::FakeCanMessage::getPointer() const
{
    return reinterpret_cast<const unsigned char *>(message);
}

// -----------------------------------------------------------------------------

void roboticslab::FakeCanMessage::setBuffer(unsigned char * buf)
{
    if (buf != nullptr)
    {
        message = reinterpret_cast<struct fake_can_msg *>(buf);
    }
}

// -----------------------------------------------------------------------------
