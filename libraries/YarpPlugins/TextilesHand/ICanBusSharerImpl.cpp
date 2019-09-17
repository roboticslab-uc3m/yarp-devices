// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// -----------------------------------------------------------------------------

unsigned int roboticslab::TextilesHand::getId()
{
    return canId;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::start()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::readyToSwitchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::switchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::enable()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::recoverFromError()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::interpretMessage(const yarp::dev::CanMessage & message)
{

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::registerSender(CanSenderDelegate * sender)
{
    return true;
}

// -----------------------------------------------------------------------------
