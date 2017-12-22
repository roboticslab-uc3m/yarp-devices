// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::setCanBusPtr(yarp::dev::ICanBus *canDevicePtr)
{

    this->canDevicePtr = canDevicePtr;
    CD_SUCCESS("Ok pointer to CAN bus device %d.\n",canId);

}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::start()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::readyToSwitchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::switchOn()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::enable()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::recoverFromError()
{

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::interpretMessage(yarp::dev::CanMessage * message)
{

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------
