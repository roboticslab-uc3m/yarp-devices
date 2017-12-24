// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PeakCanMessage.hpp"

// -----------------------------------------------------------------------------

roboticslab::PeakCanMessage::PeakCanMessage()
{
}

// -----------------------------------------------------------------------------

roboticslab::PeakCanMessage::~PeakCanMessage()
{
}

// -----------------------------------------------------------------------------

yarp::dev::CanMessage & roboticslab::PeakCanMessage::operator=(const yarp::dev::CanMessage & l)
{
    return *this;
}

// -----------------------------------------------------------------------------

unsigned int roboticslab::PeakCanMessage::getId() const
{
    return 0;
}

// -----------------------------------------------------------------------------

unsigned char roboticslab::PeakCanMessage::getLen() const
{
    return 0;
}

// -----------------------------------------------------------------------------

void roboticslab::PeakCanMessage::setLen(unsigned char len)
{
}

// -----------------------------------------------------------------------------

void roboticslab::PeakCanMessage::setId(unsigned int id)
{
}

// -----------------------------------------------------------------------------

const unsigned char * roboticslab::PeakCanMessage::getData() const
{
    return 0;
}

// -----------------------------------------------------------------------------

unsigned char * roboticslab::PeakCanMessage::getData()
{
    return 0;
}

// -----------------------------------------------------------------------------

unsigned char * roboticslab::PeakCanMessage::getPointer()
{
    return 0;
}

// -----------------------------------------------------------------------------

const unsigned char * roboticslab::PeakCanMessage::getPointer() const
{
    return 0;
}

// -----------------------------------------------------------------------------

void roboticslab::PeakCanMessage::setBuffer(unsigned char * buf)
{
}

// -----------------------------------------------------------------------------
