// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// -----------------------------------------------------------------------------

unsigned int roboticslab::TextilesHand::getId()
{
    return canId;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::initialize()
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::finalize()
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::interpretMessage(const yarp::dev::CanMessage & message)
{
    return true;

}

// -----------------------------------------------------------------------------

bool roboticslab::TextilesHand::registerSender(CanSenderDelegate * sender)
{
    return true;
}

// -----------------------------------------------------------------------------
