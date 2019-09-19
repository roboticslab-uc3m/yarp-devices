// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// -----------------------------------------------------------------------------

unsigned int roboticslab::FakeJoint::getId()
{
    return 0;
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

bool roboticslab::FakeJoint::interpretMessage(const yarp::dev::CanMessage & message)
{

    return true;

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::registerSender(CanSenderDelegate * sender)
{
    this->sender = sender;
    return true;
}

// -----------------------------------------------------------------------------
