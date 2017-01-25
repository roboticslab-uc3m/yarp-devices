// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// -----------------------------------------------------------------------------

bool teo::FakeJoint::setCanBusPtr(ICanBusHico *canDevicePtr)
{

    this->canDevicePtr = canDevicePtr;
    CD_SUCCESS("Ok pointer to CAN bus device %d.\n",canId);

}

// -----------------------------------------------------------------------------

bool teo::FakeJoint::start()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeJoint::readyToSwitchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeJoint::switchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeJoint::enable()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeJoint::recoverFromError()
{

    return true;
}

// -----------------------------------------------------------------------------

bool teo::FakeJoint::interpretMessage( can_msg * message)
{

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
