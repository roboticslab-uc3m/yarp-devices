// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <string>
#include <ColorDebug.h>

// ---------------------------- IRemoteVariables Related ----------------------------------

bool roboticslab::CanBusControlboard::getRemoteVariable(std::string key, yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());

    if (val.size() > nodes.size())
    {
        CD_ERROR("Bottle size exceeds number of nodes: %d > %zd\n", val.size(), nodes.size());
        return false;
    }
    else if (val.size() != nodes.size())
    {
        CD_WARNING("Bottle size does not match number of nodes: %d != %zd\n", val.size(), nodes.size());
    }

    bool ok = true;

    val.clear();

    for (int i = 0; i < val.size(); i++)
    {
        yarp::os::Bottle b;

        if (iRemoteVariablesRaw[i]->getRemoteVariableRaw(key, b))
        {
            val.addList() = b;
        }
        else
        {
            val.addList();
            ok = false;
        }
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::setRemoteVariable(std::string key, const yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());

    if (val.size() > nodes.size())
    {
        CD_ERROR("Bottle size exceeds number of nodes: %d > %zd\n", val.size(), nodes.size());
        return false;
    }
    else if (val.size() != nodes.size())
    {
        CD_WARNING("Bottle size does not match number of nodes: %d != %zd\n", val.size(), nodes.size());
    }

    bool ok = true;

    for (int i = 0; i < val.size(); i++)
    {
        ok &= iRemoteVariablesRaw[i]->setRemoteVariableRaw(key, *val.get(i).asList());
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusControlboard::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    CD_DEBUG("\n");
    listOfKeys->clear();

    for (unsigned int i = 0; i < nodes.size(); i++)
    {
        yarp::os::Bottle keys;

        if (iRemoteVariablesRaw[i]->getRemoteVariablesListRaw(&keys))
        {
            for (int j = 0; j < keys.size(); j++)
            {
                std::string key = keys.get(j).asString();

                if (!listOfKeys->check(key))
                {
                    // Place each key in its own list so that clients can just call check('<key>')
                    // or !find('<key>').isNull().
                    listOfKeys->addList().addString(key);
                }
            }
        }
    }

    return true;
}

// -----------------------------------------------------------------------------
