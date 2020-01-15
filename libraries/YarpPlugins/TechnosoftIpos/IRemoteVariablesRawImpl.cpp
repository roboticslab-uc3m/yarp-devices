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

    if (key == "linInterp")
    {
        if (!linInterpBuffer)
        {
            CD_ERROR("Linear interpolation mode disabled.\n");
            return false;
        }

        yarp::os::Property config;
        config.put("periodMs", linInterpBuffer->getPeriodMs());
        config.put("bufferSize", linInterpBuffer->getBufferSize());
        config.put("mode", linInterpBuffer->getType());

        val.addString(key);
        val.addList().fromString(config.toString());
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
            CD_ERROR("Currently in posd mode, cannot change config params right now.\n");
            return false;
        }

        delete linInterpBuffer;

        if (val.size() != 0)
        {
            linInterpBuffer = LinearInterpolationBuffer::createBuffer(val, vars, can->getId());
            return linInterpBuffer != nullptr;
        }

        // switch back to CSP
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

    return true;
}

// -----------------------------------------------------------------------------
