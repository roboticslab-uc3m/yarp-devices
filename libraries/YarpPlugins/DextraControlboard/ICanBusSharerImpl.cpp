// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboard.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setCanBusPtr(ICanBusHico *canDevicePtr)
{

    this->canDevicePtr = canDevicePtr;
    CD_SUCCESS("Ok pointer to CAN bus device %d.\n",canId);

}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::start()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::readyToSwitchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::switchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::enable()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::recoverFromError()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboard::interpretMessage( can_msg * message)
{

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
