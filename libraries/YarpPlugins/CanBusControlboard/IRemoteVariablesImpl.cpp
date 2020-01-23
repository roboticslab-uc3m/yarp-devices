// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

#include "ICanBusSharer.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRemoteVariable(std::string key, yarp::os::Bottle & val)
{
    CD_DEBUG("%s\n", key.c_str());

    val.clear();

    for (const auto & t : deviceMapper.getDevicesWithOffsets())
    {
        auto * iCanBusSharer = std::get<0>(t)->castToType<ICanBusSharer>();

        if (key == "id" + std::to_string(iCanBusSharer->getId()))
        {
            auto * p = std::get<0>(t)->getHandle<yarp::dev::IRemoteVariablesRaw>();
            yarp::os::Bottle b;

            if (p && p->getRemoteVariablesListRaw(&b))
            {
                bool ok = true;

                for (int j = 0; j < b.size(); j++)
                {
                    ok &= p->getRemoteVariableRaw(b.get(j).asString(), val.addList());
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

    if (val.size() != 2)
    {
        CD_ERROR("Illegal bottle format, expected single key-value list.\n");
        return false;
    }

    for (const auto & t : deviceMapper.getDevicesWithOffsets())
    {
        auto * iCanBusSharer = std::get<0>(t)->castToType<ICanBusSharer>();

        if (key == "id" + std::to_string(iCanBusSharer->getId()))
        {
            auto * p = std::get<0>(t)->getHandle<yarp::dev::IRemoteVariablesRaw>();

            if (p)
            {
                return p->setRemoteVariableRaw(val.get(0).asString(), *val.get(1).asList());
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
    for (const auto & t : deviceMapper.getDevicesWithOffsets())
    {
        auto * iCanBusSharer = std::get<0>(t)->castToType<ICanBusSharer>();
        listOfKeys->addString("id" + std::to_string(iCanBusSharer->getId()));
    }

    return true;
}

// -----------------------------------------------------------------------------
