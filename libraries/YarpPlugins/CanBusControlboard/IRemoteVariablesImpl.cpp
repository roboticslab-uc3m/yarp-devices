// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <string>
#include <ColorDebug.h>

// ---------------------------- IRemoteVariables Related ----------------------------------

bool roboticslab::CanBusControlboard::getRemoteVariable(std::string key, yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRemoteVariable(std::string key, const yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    CD_DEBUG("\n");
    listOfKeys->clear();
    // Place each key in its own list so that clients can just call check('<key>') or !find('<key>').isNull().
    return true;
}

// -----------------------------------------------------------------------------
