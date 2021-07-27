// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusSocket.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusSocket::canGetErrors(yarp::dev::CanErrors & err)
{
    std::lock_guard<std::mutex> lock(errorMutex);
    err = errors;
    return true;
}

// -----------------------------------------------------------------------------
