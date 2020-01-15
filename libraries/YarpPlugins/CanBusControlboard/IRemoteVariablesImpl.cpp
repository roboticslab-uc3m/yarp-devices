// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRemoteVariable(std::string key, yarp::os::Bottle & val)
{
    CD_DEBUG("%s\n", key.c_str());

    val.clear();

    for (int i = 0; i < nodeDevices.size(); i++)
    {
        if (key == nodeDevices[i]->key)
        {
            auto t = deviceMapper.getDevice(i);
            auto * p = std::get<0>(t)->getHandle<yarp::dev::IRemoteVariablesRaw>();
            yarp::os::Bottle b;

            if (p && p->getRemoteVariablesListRaw(&b))
            {
                bool ok = true;

                for (int j = 0; j < b.size(); j++)
                {
                    ok &= p->getRemoteVariableRaw(b.get(i).asString(), val.addList());
                }

                return ok;
            }

            CD_ERROR("Unsupported interface or failed query: \"%s\".\n", key.c_str());
            return false;
        }
    }

    CD_ERROR("Node \"%s\" not found.\n", key.c_str());
    return false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRemoteVariable(std::string key, const yarp::os::Bottle & val)
{
    CD_DEBUG("%s\n", key.c_str());

    for (int i = 0; i < nodeDevices.size(); i++)
    {
        if (key == nodeDevices[i]->key)
        {
            auto t = deviceMapper.getDevice(i);
            auto * p = std::get<0>(t)->getHandle<yarp::dev::IRemoteVariablesRaw>();

            if (p)
            {
                bool ok = true;

                for (int j = 0; j < val.size(); j++)
                {
                    auto * b = val.get(j).asList();
                    ok &= b && b->size() == 2 && b->get(1).isList()
                            && p->setRemoteVariableRaw(b->get(0).asString(), *b->get(1).asList());
                }

                return ok;
            }

            CD_ERROR("Unsupported interface: \"%s\".\n", key.c_str());
            return false;
        }
    }

    CD_ERROR("Node \"%s\" not found.\n", key.c_str());
    return false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRemoteVariablesList(yarp::os::Bottle * listOfKeys)
{
    CD_DEBUG("\n");

    listOfKeys->clear();

    // Place each key in its own list so that clients can just call check('<key>') or !find('<key>').isNull().
    for (int i = 0; i < nodeDevices.size(); i++)
    {
        listOfKeys->addString(nodeDevices[i]->key);
    }

    return true;
}

// -----------------------------------------------------------------------------
