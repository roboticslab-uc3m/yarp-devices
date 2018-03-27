// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraHand.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::setCanBusPtr(ICanBusHico *canDevicePtr)
{

    this->canDevicePtr = canDevicePtr;
    CD_SUCCESS("Ok pointer to CAN bus device %d.\n",canId);

}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::start()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::readyToSwitchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::switchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::enable()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::recoverFromError()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraHand::interpretMessage( can_msg * message)
{

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
