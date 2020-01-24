// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

#include "ICanBusSharer.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRemoteVariable(std::string key, yarp::os::Bottle & val)
{
    CD_DEBUG("%s\n", key.c_str());

    bool queryAll = key == "all";
    val.clear();

    for (const auto & t : deviceMapper.getDevicesWithOffsets())
    {
        auto * iCanBusSharer = std::get<0>(t)->castToType<ICanBusSharer>();
        yarp::os::Bottle & nodeVal = queryAll ? val.addList() : val;

        if (queryAll || key == "id" + std::to_string(iCanBusSharer->getId()))
        {
            auto * p = std::get<0>(t)->getHandle<yarp::dev::IRemoteVariablesRaw>();
            yarp::os::Bottle b;

            if (p && p->getRemoteVariablesListRaw(&b))
            {
                nodeVal.addString("id" + std::to_string(iCanBusSharer->getId()));
                bool ok = true;

                for (int j = 0; j < b.size(); j++)
                {
                    ok &= p->getRemoteVariableRaw(b.get(j).asString(), nodeVal.addList());
                }

                if (!queryAll)
                {
                    return ok;
                }
            }

            if (!queryAll)
            {
                CD_ERROR("Unsupported interface: \"%s\".\n", key.c_str());
                return false;
            }
            else if (!p)
            {
                CD_WARNING("Unsupported interface: \"id%d\".\n", iCanBusSharer->getId());
            }
        }
    }

    if (!queryAll)
    {
        CD_ERROR("Node \"%s\" not found, type e.g. \"id19\" or \"all\".\n", key.c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRemoteVariable(std::string key, const yarp::os::Bottle & val)
{
    CD_DEBUG("%s\n", key.c_str());

    if (val.size() != 2 || !val.get(1).isList())
    {
        CD_ERROR("Illegal bottle format, expected single key-value list.\n");
        return false;
    }

    bool setAll = key == "all";
    bool allOk = true;

    for (const auto & t : deviceMapper.getDevicesWithOffsets())
    {
        auto * iCanBusSharer = std::get<0>(t)->castToType<ICanBusSharer>();

        if (setAll || key == "id" + std::to_string(iCanBusSharer->getId()))
        {
            auto * p = std::get<0>(t)->getHandle<yarp::dev::IRemoteVariablesRaw>();

            if (!p)
            {
                if (!setAll)
                {
                    CD_ERROR("Unsupported interface: \"%s\".\n", key.c_str());
                    return false;
                }

                CD_WARNING("Unsupported interface: \"id%d\".\n", iCanBusSharer->getId());
            }
            else if (!p->setRemoteVariableRaw(val.get(0).asString(), *val.get(1).asList()))
            {
                if (!setAll)
                {
                    return false;
                }

                CD_WARNING("Request failed: \"id%d\".\n", iCanBusSharer->getId());
                allOk = false;
            }
            else if (!setAll)
            {
                return true;
            }
        }
    }

    if (!setAll)
    {
        CD_ERROR("Node \"%s\" not found, type e.g. \"id19\" or \"all\".\n", key.c_str());
        return false;
    }

    return allOk;
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
