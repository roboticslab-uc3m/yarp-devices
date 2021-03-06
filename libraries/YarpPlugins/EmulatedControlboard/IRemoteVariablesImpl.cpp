// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlboard.hpp"

#include <ColorDebug.h>

// ---------------------------- IRemoteVariables Related ----------------------------------

bool roboticslab::EmulatedControlboard::getRemoteVariable(std::string key, yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setRemoteVariable(std::string key, const yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    CD_DEBUG("\n");
    listOfKeys->clear();
    return true;
}

// -----------------------------------------------------------------------------
