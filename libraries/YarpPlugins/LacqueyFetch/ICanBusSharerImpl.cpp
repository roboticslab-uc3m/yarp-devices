// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

unsigned int LacqueyFetch::getId()
{
    return canId;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::initialize()
{
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::finalize()
{
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::interpretMessage(const yarp::dev::CanMessage & message)
{
    return true;
}

// -----------------------------------------------------------------------------

bool LacqueyFetch::registerSender(CanSenderDelegate * sender)
{
    this->sender = sender;
    return true;
}

// -----------------------------------------------------------------------------
