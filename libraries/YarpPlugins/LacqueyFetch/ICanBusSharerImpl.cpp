// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

// -----------------------------------------------------------------------------

unsigned int roboticslab::LacqueyFetch::getId()
{
    return canId;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::start()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::readyToSwitchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::switchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::enable()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::recoverFromError()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::interpretMessage(const yarp::dev::CanMessage & message)
{

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::registerSender(CanSenderDelegate * sender)
{
    this->sender = sender;
    return true;
}

// -----------------------------------------------------------------------------
