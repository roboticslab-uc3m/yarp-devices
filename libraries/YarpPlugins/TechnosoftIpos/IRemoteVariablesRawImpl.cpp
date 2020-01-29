// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/os/Property.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRemoteVariableRaw(std::string key, yarp::os::Bottle & val)
{
    CD_DEBUG("%s: %s\n", key.c_str(), val.toString().c_str());

    val.clear();
    val.addString(key);

    if (key == "linInterp")
    {
        yarp::os::Property & dict = val.addDict();

        if (!linInterpBuffer)
        {
            dict.put("enable", false);
        }
        else
        {
            dict.put("enable", true);
            dict.put("periodMs", linInterpBuffer->getPeriodMs());
            dict.put("mode", linInterpBuffer->getType());
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

    CD_ERROR("Unsupported key: \"%s\".\n", key.c_str());
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val)
{
    CD_DEBUG("%s\n", key.c_str());

    if (key == "linInterp")
    {
        if (val.size() == 0 || (!val.get(0).isDict() && !val.get(0).isList()))
        {
            CD_ERROR("Empty value or not a dict (canId: %d).\n", can->getId());
            return false;
        }

        // check on vars.requestedControlMode to avoid race conditions during mode switch
        if (vars.actualControlMode == VOCAB_CM_POSITION_DIRECT || vars.requestedcontrolMode == VOCAB_CM_POSITION_DIRECT)
        {
            CD_ERROR("Currently in posd mode, cannot change config params right now (canId: %d).\n", can->getId());
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
            CD_ERROR("Missing \"enable\" option (canId: %d).\n", can->getId());
            return false;
        }

        delete linInterpBuffer;
        linInterpBuffer = nullptr;

        if (dict->find("enable").asBool())
        {
            linInterpBuffer = LinearInterpolationBuffer::createBuffer(val, vars);

            if (!linInterpBuffer)
            {
                CD_ERROR("Cannot create linear interpolation buffer (canId: %d).\n", can->getId());
                return false;
            }

            CD_SUCCESS("Created %s buffer with period %d ms (canId: %d).\n",
                    linInterpBuffer->getType().c_str(), linInterpBuffer->getPeriodMs(), can->getId());
        }
        else
        {
            CD_SUCCESS("Switched back to CSP mode (canId: %d).\n", can->getId());
        }

        return true;
    }
    else if (key == "csv")
    {
        if (!val.check("enable"))
        {
            CD_ERROR("Missing \"enable\" option (canId: %d).\n", can->getId());
            return false;
        }

        bool requested = val.find("enable").asBool();

        if (requested ^ vars.enableCsv)
        {
            if (vars.actualControlMode == VOCAB_CM_VELOCITY)
            {
                CD_ERROR("Currently in vel mode, cannot change internal mode mapping right now (canId: %d).\n", can->getId());
                return false;
            }

            vars.enableCsv = requested;
        }
        else
        {
            CD_WARNING("CSV mode already enabled/disabled (canId: %d).\n", can->getId());
        }

        return true;
    }

    CD_ERROR("Unsupported key: \"%s\".\n", key.c_str());
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys)
{
    CD_DEBUG("\n");

    listOfKeys->clear();

    // Place each key in its own list so that clients can just call check('<key>') or !find('<key>').isNull().
    listOfKeys->addString("linInterp");
    listOfKeys->addString("csv");

    return true;
}

// -----------------------------------------------------------------------------
