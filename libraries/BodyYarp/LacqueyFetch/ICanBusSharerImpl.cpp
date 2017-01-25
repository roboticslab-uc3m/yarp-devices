// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::setCanBusPtr(ICanBusHico *canDevicePtr)
{

    this->canDevicePtr = canDevicePtr;
    CD_SUCCESS("Ok pointer to CAN bus device %d.\n",canId);

}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::start()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::readyToSwitchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::switchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::enable()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::recoverFromError()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::LacqueyFetch::interpretMessage( can_msg * message)
{

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
