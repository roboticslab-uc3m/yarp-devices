// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeJoint.hpp"

// ------------------ IRemoteVariablesRaw Related ----------------------------------------

bool roboticslab::FakeJoint::getRemoteVariableRaw(std::string key, yarp::os::Bottle& val)
{
    CD_WARNING("Not implemented.\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val)
{
    CD_WARNING("Not implemented.\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeJoint::getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys)
{
    CD_DEBUG("\n");
    listOfKeys->clear();
    return true;
}

// -----------------------------------------------------------------------------
