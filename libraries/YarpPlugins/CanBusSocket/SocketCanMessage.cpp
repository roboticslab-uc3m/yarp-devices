// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <SocketCanMessage.hpp>

// -----------------------------------------------------------------------------

roboticslab::SocketCanMessage::SocketCanMessage()
{
}

// -----------------------------------------------------------------------------

roboticslab::SocketCanMessage::~SocketCanMessage()
{
}

// -----------------------------------------------------------------------------

yarp::dev::CanMessage & roboticslab::SocketCanMessage::operator=(const yarp::dev::CanMessage & l)
{
    return *this;
}

// -----------------------------------------------------------------------------

unsigned int roboticslab::SocketCanMessage::getId() const
{
    return 0;
}

// -----------------------------------------------------------------------------

unsigned char roboticslab::SocketCanMessage::getLen() const
{
    return 0;
}

// -----------------------------------------------------------------------------

void roboticslab::SocketCanMessage::setLen(unsigned char len)
{
}

// -----------------------------------------------------------------------------

void roboticslab::SocketCanMessage::setId(unsigned int id)
{
}

// -----------------------------------------------------------------------------

const unsigned char * roboticslab::SocketCanMessage::getData() const
{
    return 0;
}

// -----------------------------------------------------------------------------

unsigned char * roboticslab::SocketCanMessage::getData()
{
    return 0;
}

// -----------------------------------------------------------------------------

unsigned char * roboticslab::SocketCanMessage::getPointer()
{
    return 0;
}

// -----------------------------------------------------------------------------

const unsigned char * roboticslab::SocketCanMessage::getPointer() const
{
    return 0;
}

// -----------------------------------------------------------------------------

void roboticslab::SocketCanMessage::setBuffer(unsigned char * buf)
{
}

// -----------------------------------------------------------------------------
