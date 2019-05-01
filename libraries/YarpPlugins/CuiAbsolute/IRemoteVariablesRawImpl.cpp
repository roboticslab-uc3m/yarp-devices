// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

// ------------------ IRemoteVariablesRaw Related ----------------------------------------

bool roboticslab::CuiAbsolute::getRemoteVariableRaw(std::string key, yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::CuiAbsolute::getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys)
{
    CD_DEBUG("\n");
    listOfKeys->clear();
    return true;
}

// -----------------------------------------------------------------------------
