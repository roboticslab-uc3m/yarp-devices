// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

namespace
{
    bool setSingleKeyValuePair(const std::string & key, const yarp::os::Bottle & val, const DeviceMapper & mapper)
    {
        if (val.size() != 2 || !val.get(0).isString())
        {
            yCError(CBB) << "Illegal bottle format, two elements expected: string key and value:" << val.toString();
            return false;
        }

        bool setAll = key == "all";
        bool allOk = true;

        for (const auto & rawDevice : mapper.getDevices())
        {
            const auto id = rawDevice->getId();

            if (!id.empty() && (setAll || key == id))
            {
                auto * p = rawDevice->getHandle<yarp::dev::IRemoteVariablesRaw>();

                if (!p)
                {
                    if (!setAll)
                    {
                        yCError(CBB) << "Unsupported interface:" << key;
                        return false;
                    }

                    yCWarning(CBB) << "Unsupported interface:" << id;
                }
                else if (!p->setRemoteVariableRaw(val.get(0).asString(), val.tail()))
                {
                    if (!setAll)
                    {
                        return false;
                    }

                    yCWarning(CBB) << "Request failed:" << id;
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
            yCError(CBB) << "Node" << key << "not found, type e.g. \"ID19\" or \"all\"";
            return false;
        }

        return allOk;
    }
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getRemoteVariable(std::string key, yarp::os::Bottle & val)
#else
bool CanBusBroker::getRemoteVariable(std::string key, yarp::os::Bottle & val)
#endif
{
    bool queryAll = key == "all";
    val.clear();

    for (const auto & rawDevice : deviceMapper.getDevices())
    {
        const auto id = rawDevice->getId();

        if (!id.empty() && (queryAll || key == id))
        {
            auto * p = rawDevice->getHandle<yarp::dev::IRemoteVariablesRaw>();
            yarp::os::Bottle b;

            if (p && p->getRemoteVariablesListRaw(&b) && b.size() != 0)
            {
                // additional nesting because of controlboardremapper+yarpmotorgui
                auto & nodeVal = val.addList();

                if (queryAll)
                {
                    nodeVal.addString(id);
                }

                bool ok = true;

                for (int j = 0; j < b.size(); j++)
                {
                    ok &= p->getRemoteVariableRaw(b.get(j).asString(), nodeVal.addList());
                }

                if (!queryAll)
                {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
                    return ok;
#endif
                }
            }

            if (!queryAll)
            {
                yCError(CBB) << "Unsupported interface:" << key;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                return yarp::dev::ReturnValue_error_method_failed;
#else
                return false;
#endif
            }
            else if (!p)
            {
                yCWarning(CBB) << "Unsupported interface:" << id;
            }
        }
    }

    if (!queryAll)
    {
        yCError(CBB) << "Node" << key << "not found, type e.g. \"ID19\" or \"all\"";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setRemoteVariable(std::string key, const yarp::os::Bottle & val)
#else
bool CanBusBroker::setRemoteVariable(std::string key, const yarp::os::Bottle & val)
#endif
{
    if (key == "multi")
    {
        bool ok = true;

        for (int i = 0; i < val.size(); i++)
        {
            if (!val.get(i).isList())
            {
                yCError(CBB) << "Not a list:", val.get(i).toString();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                return yarp::dev::ReturnValue_error_method_failed;
#else
                return false;
#endif
            }

            const auto * nestedVal = val.get(i).asList();

            if (nestedVal->size() < 2 || !nestedVal->get(0).isString())
            {
                yCError(CBB) << "Illegal bottle format, expected string ID and values:" << nestedVal->toString();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                return yarp::dev::ReturnValue_error_method_failed;
#else
                return false;
#endif
            }

            const auto id = nestedVal->get(0).asString();

            if (id == "all")
            {
                yCError(CBB) << "Cannot set all node vars in multi mode";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                return yarp::dev::ReturnValue_error_method_failed;
#else
                return false;
#endif
            }

            for (int i = 1; i < nestedVal->size(); i++)
            {
                if (!nestedVal->get(i).isList())
                {
                    yCError(CBB) << "Not a list:" << nestedVal->get(i).toString();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                    return yarp::dev::ReturnValue_error_method_failed;
#else
                    return false;
#endif
                }

                ok &= setSingleKeyValuePair(id, *nestedVal->get(i).asList(), deviceMapper);
            }
        }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
        return ok;
#endif
    }

    if (val.size() == 0)
    {
        yCError(CBB) << "Empty value list";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    if (val.get(0).isList())
    {
        bool ok = true;

        for (int i = 0; i < val.size(); i++)
        {
            if (!val.get(i).isList())
            {
                yCError(CBB) << "Not a list:" << val.get(i).toString();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                return yarp::dev::ReturnValue_error_method_failed;
#else
                return false;
#endif
            }

            ok &= setSingleKeyValuePair(key, *val.get(i).asList(), deviceMapper);
        }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return ok
            ? yarp::dev::ReturnValue_ok
            : yarp::dev::ReturnValue_error_method_failed;
#else
        return ok;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return setSingleKeyValuePair(key, val, deviceMapper)
        ? yarp::dev::ReturnValue_ok
        : yarp::dev::ReturnValue_error_method_failed;
#else
    return setSingleKeyValuePair(key, val, deviceMapper);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getRemoteVariablesList(yarp::os::Bottle * listOfKeys)
#else
bool CanBusBroker::getRemoteVariablesList(yarp::os::Bottle * listOfKeys)
#endif
{
    listOfKeys->clear();

    // Place each key in its own list so that clients can just call check('<key>') or !find('<key>').isNull().
    for (const auto & rawDevice : deviceMapper.getDevices())
    {
        const auto id = rawDevice->getId();

        if (!id.empty())
        {
            listOfKeys->addString(id);
        }
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
