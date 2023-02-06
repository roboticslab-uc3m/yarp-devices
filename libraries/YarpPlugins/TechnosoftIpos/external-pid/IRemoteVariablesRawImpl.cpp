// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getRemoteVariableRaw(std::string key, yarp::os::Bottle & val)
{
    yCITrace(IPOS, id(), "%s: %s", key.c_str(), val.toString().c_str());

    val.addString(key);

    if (key == "enableCsv")
    {
        val.addInt32(enableCsv ? 1 : 0);
    }
    else
    {
        yCIError(IPOS, id()) << "Unsupported key:" << key;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val)
{
    yCITrace(IPOS, id(), "%s", key.c_str());

    if (key == "enableCsv")
    {
        auto requested = val.get(0).asBool();

        if (requested ^ enableCsv)
        {
            if (actualControlMode == VOCAB_CM_VELOCITY || requestedcontrolMode == VOCAB_CM_VELOCITY)
            {
                yCIError(IPOS, id()) << "Currently in vel mode, cannot change internal mode mapping right now";
                return false;
            }

            enableCsv = requested;
            yCIInfo(IPOS, id()) << "CSV mode" << (requested ? "enabled" : "disabled");
        }
    }
    else
    {
        yCIError(IPOS, id()) << "Unsupported key:" << key;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys)
{
    yCITrace(IPOS, id());

    listOfKeys->clear();
    listOfKeys->addString("enableCsv");

    return true;
}

// -----------------------------------------------------------------------------
