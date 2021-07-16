// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusSocket.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusSocket::canSetBaudRate(unsigned int rate)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canGetBaudRate(unsigned int * rate)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canIdAdd(unsigned int id)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canIdDelete(unsigned int id)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait)
{
    return false;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait)
{
    return false;
}

// -----------------------------------------------------------------------------
