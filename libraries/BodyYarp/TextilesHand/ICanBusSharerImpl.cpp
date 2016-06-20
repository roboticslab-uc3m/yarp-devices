// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// -----------------------------------------------------------------------------

bool teo::TextilesHand::setCanBusPtr(CanBusHico *canDevicePtr)
{

    this->canDevicePtr = canDevicePtr;
    CD_SUCCESS("Ok pointer to CAN bus device %d.\n",canId);

}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::start()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::readyToSwitchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::switchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::enable()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::recoverFromError()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::TextilesHand::interpretMessage( can_msg * message)
{

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
