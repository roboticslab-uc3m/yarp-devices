// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusSocket.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::CanBusSocket::canSetBaudRate(unsigned int rate)
{
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusSocket::canGetBaudRate(unsigned int * rate)
{
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusSocket::canIdAdd(unsigned int id)
{
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusSocket::canIdDelete(unsigned int id)
{
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusSocket::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusSocket::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    return false;
}

// -----------------------------------------------------------------------------
