// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeCanMessage.hpp"

#include <cstring> // std::memcpy

using namespace roboticslab;

// -----------------------------------------------------------------------------

FakeCanMessage::FakeCanMessage()
{
    message = nullptr;
}

// -----------------------------------------------------------------------------

FakeCanMessage::~FakeCanMessage()
{
}

// -----------------------------------------------------------------------------

yarp::dev::CanMessage & FakeCanMessage::operator=(const yarp::dev::CanMessage & l)
{
    const FakeCanMessage & tmp = dynamic_cast<const FakeCanMessage &>(l);
    std::memcpy(message, tmp.message, sizeof(struct fake_can_msg));
    return *this;
}

// -----------------------------------------------------------------------------

unsigned int FakeCanMessage::getId() const
{
    return message->id;
}

// -----------------------------------------------------------------------------

unsigned char FakeCanMessage::getLen() const
{
    return message->dlc;
}

// -----------------------------------------------------------------------------

void FakeCanMessage::setLen(unsigned char len)
{
    message->dlc = len;
}

// -----------------------------------------------------------------------------

void FakeCanMessage::setId(unsigned int id)
{
    message->id = id;
}

// -----------------------------------------------------------------------------

const unsigned char * FakeCanMessage::getData() const
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * FakeCanMessage::getData()
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * FakeCanMessage::getPointer()
{
    return reinterpret_cast<unsigned char *>(message);
}

// -----------------------------------------------------------------------------

const unsigned char * FakeCanMessage::getPointer() const
{
    return reinterpret_cast<const unsigned char *>(message);
}

// -----------------------------------------------------------------------------

void FakeCanMessage::setBuffer(unsigned char * buf)
{
    if (buf != nullptr)
    {
        message = reinterpret_cast<struct fake_can_msg *>(buf);
    }
}

// -----------------------------------------------------------------------------
