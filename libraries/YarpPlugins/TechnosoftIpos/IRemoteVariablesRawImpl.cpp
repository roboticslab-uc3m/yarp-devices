// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/os/Log.h>
#include <yarp/os/Property.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRemoteVariableRaw(std::string key, yarp::os::Bottle & val)
{
    yTrace("%s: %s", key.c_str(), val.toString().c_str());

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

    yError("Unsupported key: \"%s\"", key.c_str());
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val)
{
    yTrace("%s", key.c_str());

    if (key == "linInterp")
    {
        if (val.size() == 0 || (!val.get(0).isDict() && !val.get(0).isList()))
        {
            yError("Empty value or not a dict (canId %d)", can->getId());
            return false;
        }

        // check on vars.requestedControlMode to avoid race conditions during mode switch
        if (vars.actualControlMode == VOCAB_CM_POSITION_DIRECT || vars.requestedcontrolMode == VOCAB_CM_POSITION_DIRECT)
        {
            yError("Currently in posd mode, cannot change config params right now (canId %d)", can->getId());
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
            yError("Missing \"enable\" option (canId %d)", can->getId());
            return false;
        }

        delete ipBuffer;
        ipBuffer = nullptr;

        if (dict->find("enable").asBool())
        {
            ipBuffer = createInterpolationBuffer(val, vars);

            if (!ipBuffer)
            {
                yError("Cannot create ip buffer (canId %d)", can->getId());
                return false;
            }

            yInfo("Created %s buffer with %d points and period %d ms (canId %d)",
                ipBuffer->getType().c_str(), ipBuffer->getBufferSize(), ipBuffer->getPeriodMs(), can->getId());
        }
        else
        {
            yInfo("Switched back to CSP mode (canId %d)", can->getId());
        }

        return true;
    }
    else if (key == "csv")
    {
        if (!val.check("enable"))
        {
            yError("Missing \"enable\" option (canId %d)", can->getId());
            return false;
        }

        bool requested = val.find("enable").asBool();

        if (requested ^ vars.enableCsv)
        {
            if (vars.actualControlMode == VOCAB_CM_VELOCITY)
            {
                yError("Currently in vel mode, cannot change internal mode mapping right now (canId %d)", can->getId());
                return false;
            }

            vars.enableCsv = requested;
        }
        else
        {
            yWarning("CSV mode already enabled/disabled (canId %d)", can->getId());
        }

        return true;
    }

    yError("Unsupported key: \"%s\"", key.c_str());
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys)
{
    yTrace("");

    listOfKeys->clear();

    // Place each key in its own list so that clients can just call check('<key>') or !find('<key>').isNull().
    listOfKeys->addString("linInterp");
    listOfKeys->addString("csv");

    return true;
}

// -----------------------------------------------------------------------------
