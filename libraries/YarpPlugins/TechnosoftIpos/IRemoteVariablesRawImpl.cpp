// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IRemoteVariablesRaw Related ----------------------------------------

bool roboticslab::TechnosoftIpos::getRemoteVariableRaw(std::string key, yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys)
{
    CD_DEBUG("\n");
    listOfKeys->clear();
    listOfKeys->addString("pvtPoints");
    return true;
}

// -----------------------------------------------------------------------------
