// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRemoteVariableRaw(std::string key, yarp::os::Bottle & val)
{
#if YARP_VERSION_MINOR >= 6
    yCITrace(IPOS, id(), "%s: %s", key.c_str(), val.toString().c_str());
#else
    yCTrace(IPOS, "%s: %s", key.c_str(), val.toString().c_str());
#endif

    val.clear();
    val.addString(key);

    if (key == "linInterp")
    {
        yarp::os::Property & dict = val.addDict();

        if (!ipBuffer)
        {
            dict.put("enable", false);
        }
        else
        {
            dict.put("enable", true);
            dict.put("periodMs", ipBuffer->getPeriodMs());
            dict.put("mode", ipBuffer->getType());
        }

        return true;
    }
    else if (key == "csv")
    {
        yarp::os::Bottle & list = val.addList();
        list.addString("enable");
        list.addInt8(vars.enableCsv);
        return true;
    }

#if YARP_VERSION_MINOR >= 6
    yCIError(IPOS, id()) << "Unsupported key:" << key;
#else
    yCError(IPOS, "Unsupported key: \"%s\"", key.c_str());
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val)
{
#if YARP_VERSION_MINOR >= 6
    yCITrace(IPOS, id(), "%s", key.c_str());
#else
    yCTrace(IPOS, "%s", key.c_str());
#endif

    if (key == "linInterp")
    {
        if (val.size() == 0 || (!val.get(0).isDict() && !val.get(0).isList()))
        {
#if YARP_VERSION_MINOR >= 6
            yCIError(IPOS, id()) << "Empty value or not a dict";
#else
            yCError(IPOS, "Empty value or not a dict (canId %d)", can->getId());
#endif
            return false;
        }

        // check on vars.requestedControlMode to avoid race conditions during mode switch
        if (vars.actualControlMode == VOCAB_CM_POSITION_DIRECT || vars.requestedcontrolMode == VOCAB_CM_POSITION_DIRECT)
        {
#if YARP_VERSION_MINOR >= 6
            yCIError(IPOS, id()) << "Currently in posd mode, cannot change config params right now";
#else
            yCError(IPOS, "Currently in posd mode, cannot change config params right now (canId %d)", can->getId());
#endif
            return false;
        }

        yarp::os::Searchable * dict;

        if (val.get(0).isDict())
        {
            dict = val.get(0).asDict(); // C++ API
        }
        else
        {
            dict = val.get(0).asList(); // CLI (RPC via terminal)
        }

        if (!dict->check("enable"))
        {
#if YARP_VERSION_MINOR >= 6
            yCIError(IPOS, id()) << "Missing \"enable\" option";
#else
            yCError(IPOS, "Missing \"enable\" option (canId %d)", can->getId());
#endif
            return false;
        }

        delete ipBuffer;
        ipBuffer = nullptr;

        if (dict->find("enable").asBool())
        {
            ipBuffer = createInterpolationBuffer(val, vars);

            if (!ipBuffer)
            {
#if YARP_VERSION_MINOR >= 6
                yCIError(IPOS, id()) << "Cannot create ip buffer";
#else
                yCError(IPOS, "Cannot create ip buffer (canId %d)", can->getId());
#endif
                return false;
            }

#if YARP_VERSION_MINOR >= 6
            yCIInfo(IPOS, id()) << "Created" << ipBuffer->getType() << "buffer with" << ipBuffer->getBufferSize()
                                << "points and period" << ipBuffer->getPeriodMs() << "ms";
#else
            yCInfo(IPOS, "Created %s buffer with %d points and period %d ms (canId %d)",
                ipBuffer->getType().c_str(), ipBuffer->getBufferSize(), ipBuffer->getPeriodMs(), can->getId());
#endif
        }
        else
        {
#if YARP_VERSION_MINOR >= 6
            yCIInfo(IPOS, id()) << "Switched back to CSP mode";
#else
            yCInfo(IPOS, "Switched back to CSP mode (canId %d)", can->getId());
#endif
        }

        return true;
    }
    else if (key == "csv")
    {
        if (!val.check("enable"))
        {
#if YARP_VERSION_MINOR >= 6
            yCIError(IPOS, id()) << "Missing \"enable\" option";
#else
            yCError(IPOS, "Missing \"enable\" option (canId %d)", can->getId());
#endif
            return false;
        }

        bool requested = val.find("enable").asBool();

        if (requested ^ vars.enableCsv)
        {
            if (vars.actualControlMode == VOCAB_CM_VELOCITY)
            {
#if YARP_VERSION_MINOR >= 6
                yCIError(IPOS, id()) << "Currently in vel mode, cannot change internal mode mapping right now";
#else
                yCError(IPOS, "Currently in vel mode, cannot change internal mode mapping right now (canId %d)", can->getId());
#endif
                return false;
            }

            vars.enableCsv = requested;
        }
        else
        {
#if YARP_VERSION_MINOR >= 6
            yCIWarning(IPOS, id()) << "CSV mode already enabled/disabled";
#else
            yCWarning(IPOS, "CSV mode already enabled/disabled (canId %d)", can->getId());
#endif
        }

        return true;
    }

#if YARP_VERSION_MINOR >= 6
    yCIError(IPOS, id()) << "Unsupported key:" << key;
#else
    yCError(IPOS, "Unsupported key: \"%s\"", key.c_str());
#endif
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys)
{
#if YARP_VERSION_MINOR >= 6
    yCITrace(IPOS, id());
#else
    yCTrace(IPOS, "");
#endif

    listOfKeys->clear();

    // Place each key in its own list so that clients can just call check('<key>') or !find('<key>').isNull().
    listOfKeys->addString("linInterp");
    listOfKeys->addString("csv");

    return true;
}

// -----------------------------------------------------------------------------
