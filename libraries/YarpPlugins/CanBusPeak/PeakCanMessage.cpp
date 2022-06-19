// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PeakCanMessage.hpp"

#include <cstring> // memcpy

using namespace roboticslab;

// -----------------------------------------------------------------------------

PeakCanMessage::PeakCanMessage()
{
    message = nullptr;
}

// -----------------------------------------------------------------------------

PeakCanMessage::~PeakCanMessage()
{
}

// -----------------------------------------------------------------------------

yarp::dev::CanMessage & PeakCanMessage::operator=(const yarp::dev::CanMessage & l)
{
    const auto & tmp = dynamic_cast<const PeakCanMessage &>(l);
    std::memcpy(message, tmp.message, sizeof(struct pcanfd_msg));
    return *this;
}

// -----------------------------------------------------------------------------

unsigned int PeakCanMessage::getId() const
{
    return message->id;
}

// -----------------------------------------------------------------------------

unsigned char PeakCanMessage::getLen() const
{
    return message->data_len;
}

// -----------------------------------------------------------------------------

void PeakCanMessage::setLen(unsigned char len)
{
    message->data_len = len;
}

// -----------------------------------------------------------------------------

void PeakCanMessage::setId(unsigned int id)
{
    message->id = id;
}

// -----------------------------------------------------------------------------

const unsigned char * PeakCanMessage::getData() const
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * PeakCanMessage::getData()
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * PeakCanMessage::getPointer()
{
    return reinterpret_cast<unsigned char *>(message);
}

// -----------------------------------------------------------------------------

const unsigned char * PeakCanMessage::getPointer() const
{
    return reinterpret_cast<const unsigned char *>(message);
}

// -----------------------------------------------------------------------------

void PeakCanMessage::setBuffer(unsigned char * buf)
{
    if (buf)
    {
        message = reinterpret_cast<struct pcanfd_msg *>(buf);
    }
}

// -----------------------------------------------------------------------------
