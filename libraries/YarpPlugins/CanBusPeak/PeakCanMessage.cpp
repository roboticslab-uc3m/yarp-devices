// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PeakCanMessage.hpp"

#include <cstring>  // memcpy

// -----------------------------------------------------------------------------

roboticslab::PeakCanMessage::PeakCanMessage()
{
    message = 0;
}

// -----------------------------------------------------------------------------

roboticslab::PeakCanMessage::~PeakCanMessage()
{
}

// -----------------------------------------------------------------------------

yarp::dev::CanMessage & roboticslab::PeakCanMessage::operator=(const yarp::dev::CanMessage & l)
{
    const PeakCanMessage & tmp = dynamic_cast<const PeakCanMessage &>(l);
    std::memcpy(message, tmp.message, sizeof(struct pcanfd_msg));
    return *this;
}

// -----------------------------------------------------------------------------

unsigned int roboticslab::PeakCanMessage::getId() const
{
    return message->id;
}

// -----------------------------------------------------------------------------

unsigned char roboticslab::PeakCanMessage::getLen() const
{
    return message->data_len;
}

// -----------------------------------------------------------------------------

void roboticslab::PeakCanMessage::setLen(unsigned char len)
{
    message->data_len = len;
}

// -----------------------------------------------------------------------------

void roboticslab::PeakCanMessage::setId(unsigned int id)
{
    message->id = id;
}

// -----------------------------------------------------------------------------

const unsigned char * roboticslab::PeakCanMessage::getData() const
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * roboticslab::PeakCanMessage::getData()
{
    return message->data;
}

// -----------------------------------------------------------------------------

unsigned char * roboticslab::PeakCanMessage::getPointer()
{
    return reinterpret_cast<unsigned char *>(message);
}

// -----------------------------------------------------------------------------

const unsigned char * roboticslab::PeakCanMessage::getPointer() const
{
    return reinterpret_cast<const unsigned char *>(message);
}

// -----------------------------------------------------------------------------

void roboticslab::PeakCanMessage::setBuffer(unsigned char * buf)
{
    if (buf != 0)
    {
        message = reinterpret_cast<struct pcanfd_msg *>(buf);
    }
}

// -----------------------------------------------------------------------------
