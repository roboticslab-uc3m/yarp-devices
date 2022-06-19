// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRemoteVariableRaw(std::string key, yarp::os::Bottle & val)
{
    yCITrace(IPOS, id(), "%s: %s", key.c_str(), val.toString().c_str());

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

    yCIError(IPOS, id()) << "Unsupported key:" << key;
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val)
{
    yCITrace(IPOS, id(), "%s", key.c_str());

    if (key == "linInterp")
    {
        if (val.size() == 0 || (!val.get(0).isDict() && !val.get(0).isList()))
        {
            yCIError(IPOS, id()) << "Empty value or not a dict";
            return false;
        }

        // check on vars.requestedControlMode to avoid race conditions during mode switch
        if (vars.actualControlMode == VOCAB_CM_POSITION_DIRECT || vars.requestedcontrolMode == VOCAB_CM_POSITION_DIRECT)
        {
            yCIError(IPOS, id()) << "Currently in posd mode, cannot change config params right now";
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
            yCIError(IPOS, id()) << "Missing \"enable\" option";
            return false;
        }

        delete ipBuffer;
        ipBuffer = nullptr;

        if (dict->find("enable").asBool())
        {
            ipBuffer = createInterpolationBuffer(val, vars);

            if (!ipBuffer)
            {
                yCIError(IPOS, id()) << "Cannot create ip buffer";
                return false;
            }

            yCIInfo(IPOS, id()) << "Created" << ipBuffer->getType() << "buffer with" << ipBuffer->getBufferSize()
                                << "points and period" << ipBuffer->getPeriodMs() << "ms";
        }
        else
        {
            yCIInfo(IPOS, id()) << "Switched back to CSP mode";
        }

        return true;
    }
    else if (key == "csv")
    {
        if (!val.check("enable"))
        {
            yCIError(IPOS, id()) << "Missing \"enable\" option";
            return false;
        }

        bool requested = val.find("enable").asBool();

        if (requested ^ vars.enableCsv)
        {
            if (vars.actualControlMode == VOCAB_CM_VELOCITY)
            {
                yCIError(IPOS, id()) << "Currently in vel mode, cannot change internal mode mapping right now";
                return false;
            }

            vars.enableCsv = requested;
        }
        else
        {
            yCIWarning(IPOS, id()) << "CSV mode already enabled/disabled";
        }

        return true;
    }

    yCIError(IPOS, id()) << "Unsupported key:" << key;
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys)
{
    yCITrace(IPOS, id());

    listOfKeys->clear();

    // Place each key in its own list so that clients can just call check('<key>') or !find('<key>').isNull().
    listOfKeys->addString("linInterp");
    listOfKeys->addString("csv");

    return true;
}

// -----------------------------------------------------------------------------
