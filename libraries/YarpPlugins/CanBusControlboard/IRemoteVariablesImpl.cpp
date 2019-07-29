// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <string>
#include <ColorDebug.h>

// ---------------------------- IRemoteVariables Related ----------------------------------

bool roboticslab::CanBusControlboard::getRemoteVariable(std::string key, yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());

    val.clear();

    if (key == "linInterpPeriodMs")
    {
        val.addInt32(linInterpPeriodMs);
    }
    else if (key == "linInterpBufferSize")
    {
        val.addInt32(linInterpBufferSize);
    }
    else if (key == "linInterpMode")
    {
        val.addString(linInterpMode);
    }
    else
    {
        CD_ERROR("Unsupported key: %s.\n", key.c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRemoteVariable(std::string key, const yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());

    if (key != "linInterpPeriodMs" && key != "linInterpBufferSize" && key != "linInterpMode")
    {
        CD_ERROR("Unsupported key: %s.\n", key.c_str());
        return false;
    }

    int modes[motorIds.size()];

    if (!getControlModes(motorIds.size(), motorIds.data(), modes))
    {
        CD_ERROR("Unable to retrieve control modes.\n");
        return false;
    }

    for (int i = 0; i < motorIds.size(); i++)
    {
        if (modes[i] == VOCAB_CM_POSITION_DIRECT)
        {
            CD_ERROR("CAN ID %d currently in posd mode, cannot change config params right now.\n", nodes[motorIds[i]]->getValue("canId").asInt32());
            return false;
        }
    }

    for (int i = 0; i < motorIds.size(); i++)
    {
        iRemoteVariablesRaw[motorIds[i]]->setRemoteVariableRaw(key, val);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    CD_DEBUG("\n");

    listOfKeys->clear();

    // Place each key in its own list so that clients can just call check('<key>') or !find('<key>').isNull().
    listOfKeys->addString("linInterpPeriodMs");
    listOfKeys->addString("linInterpBufferSize");
    listOfKeys->addString("linInterpMode");

    return true;
}

// -----------------------------------------------------------------------------
