// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/os/Property.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRemoteVariableRaw(std::string key, yarp::os::Bottle & val)
{
    CD_DEBUG("%s\n", key.c_str());

    val.clear();
    val.addString(key);

    if (key == "linInterp")
    {
        if (!linInterpBuffer)
        {
            val.addList().addInt8(false);
            return true;
        }

        yarp::os::Property config;
        config.put("periodMs", linInterpBuffer->getPeriodMs());
        config.put("bufferSize", linInterpBuffer->getBufferSize());
        config.put("mode", linInterpBuffer->getType());

        val.addList().fromString(config.toString());
        return true;
    }
    else if (key == "csv")
    {
        val.addList().addInt8(vars.enableCsv);
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
        if (vars.actualControlMode == VOCAB_CM_POSITION_DIRECT)
        {
            CD_ERROR("Currently in posd mode, cannot change config params right now (canId: %d).\n", can->getId());
            return false;
        }

        delete linInterpBuffer;

        if (val.size() != 0)
        {
            linInterpBuffer = LinearInterpolationBuffer::createBuffer(val, vars, can->getId());
            return linInterpBuffer != nullptr;
        }

        CD_SUCCESS("Switched back to CSP mode (canId: %d).\n", can->getId());
        return true;
    }
    else if (key == "csv")
    {
        if (val.size() != 1)
        {
            CD_ERROR("One element required (true/false), %d given (canId: %d).\n", val.size(), can->getId());
            return false;
        }

        bool requested = val.get(0).asBool();

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
