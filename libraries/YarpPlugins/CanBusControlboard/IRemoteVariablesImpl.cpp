// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    bool setSingleKeyValuePair(const std::string & key, const yarp::os::Bottle & val, const DeviceMapper & mapper)
    {
        if (val.size() != 2 || !val.get(0).isString() || !val.get(1).isList())
        {
            yCError(CBCB) << "Illegal bottle format, expected string key and list value";
            return false;
        }

        bool setAll = key == "all";
        bool allOk = true;

        for (const auto & [rawDevice, offset] : mapper.getDevicesWithOffsets())
        {
            const auto id = rawDevice->getId();

            if (!id.empty() && (setAll || key == id))
            {
                auto * p = rawDevice->getHandle<yarp::dev::IRemoteVariablesRaw>();

                if (!p)
                {
                    if (!setAll)
                    {
                        yCError(CBCB) << "Unsupported interface:" << key;
                        return false;
                    }

                    yCWarning(CBCB) << "Unsupported interface:" << id;
                }
                else if (!p->setRemoteVariableRaw(val.get(0).asString(), *val.get(1).asList()))
                {
                    if (!setAll)
                    {
                        return false;
                    }

                    yCWarning(CBCB) << "Request failed:" << id;
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
            yCError(CBCB) << "Node" << key << "not found, type e.g. \"ID19\" or \"all\"";
            return false;
        }

        return allOk;
    }
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRemoteVariable(std::string key, yarp::os::Bottle & val)
{
    yCTrace(CBCB, "%s", key.c_str());

    bool queryAll = key == "all";
    val.clear();

    for (const auto & [rawDevice, offset] : deviceMapper.getDevicesWithOffsets())
    {
        const auto id = rawDevice->getId();
        yarp::os::Bottle & nodeVal = queryAll ? val.addList() : val;

        if (!id.empty() && (queryAll || key == id))
        {
            auto * p = rawDevice->getHandle<yarp::dev::IRemoteVariablesRaw>();
            yarp::os::Bottle b;

            if (p && p->getRemoteVariablesListRaw(&b))
            {
                nodeVal.addString(id);
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
                yCError(CBCB) << "Unsupported interface:" << key;
                return false;
            }
            else if (!p)
            {
                yCWarning(CBCB) << "Unsupported interface:" << id;
            }
        }
    }

    if (!queryAll)
    {
        yCError(CBCB) << "Node" << key << "not found, type e.g. \"ID19\" or \"all\"";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRemoteVariable(std::string key, const yarp::os::Bottle & val)
{
    yCTrace(CBCB, "%s: %s", key.c_str(), val.toString().c_str());

    if (key == "multi")
    {
        bool ok = true;

        for (int i = 0; i < val.size(); i++)
        {
            if (!val.get(i).isList())
            {
                yCError(CBCB) << "Not a list:", val.get(i).toString();
                return false;
            }

            yarp::os::Bottle * b = val.get(i).asList();

            if (b->size() != 2 || !b->get(0).isString() || !b->get(1).isList())
            {
                yCError(CBCB) << "Illegal bottle format, expected string key and list value:" << b->toString();
                return false;
            }

            if (b->get(0).asString() == "all")
            {
                yCError(CBCB) << "Cannot set all node vars in multi mode";
                return false;
            }

            ok &= setRemoteVariable(b->get(0).asString(), *b->get(1).asList());
        }

        return ok;
    }

    if (val.size() == 0)
    {
        yCError(CBCB) << "Empty value list";
        return false;
    }

    if (val.get(0).isList())
    {
        bool ok = true;

        for (int i = 0; i < val.size(); i++)
        {
            if (!val.get(i).isList())
            {
                yCError(CBCB) << "Not a list:" << val.get(i).toString();
                return false;
            }

            ok &= setSingleKeyValuePair(key, *val.get(i).asList(), deviceMapper);
        }

        return ok;
    }

    return setSingleKeyValuePair(key, val, deviceMapper);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRemoteVariablesList(yarp::os::Bottle * listOfKeys)
{
    yCTrace(CBCB, "");

    listOfKeys->clear();

    // Place each key in its own list so that clients can just call check('<key>') or !find('<key>').isNull().
    for (const auto & [rawDevice, offset] : deviceMapper.getDevicesWithOffsets())
    {
        const auto id = rawDevice->getId();

        if (!id.empty())
        {
            listOfKeys->addString(id);
        }
    }

    return true;
}

// -----------------------------------------------------------------------------
