// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IRemoteVariablesRaw Related ----------------------------------------

bool roboticslab::TechnosoftIpos::getRemoteVariableRaw(std::string key, yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());

    val.clear();

    if (key == "linInterpPeriodMs")
    {
        val.addInt32(linInterpBuffer->getPeriod());
    }
    else if (key == "linInterpBufferSize")
    {
        val.addInt32(linInterpBuffer->getBufferSize());
    }
    else if (key == "linInterpMode")
    {
        val.addString(linInterpBuffer->getType());
    }
    else
    {
        CD_ERROR("Unsupported key: %s.\n", key.c_str());
        return false;
    }

    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val)
{
    CD_DEBUG("%s\n", key.c_str());

    int mode;

    if (!getControlModeRaw1(&mode))
    {
        CD_ERROR("Unable to query control mode.\n");
        return false;
    }

    if (mode == VOCAB_CM_POSITION_DIRECT)
    {
        CD_ERROR("Currently in posd mode, cannot change config params right now.\n");
        return false;
    }

    if (key == "linInterpPeriodMs")
    {
        linInterpBuffer->setPeriod(val.get(0).asInt32());
    }
    else if (key == "linInterpBufferSize")
    {
        linInterpBuffer->setBufferSize(val.get(0).asInt32());
    }
    else if (key == "linInterpMode")
    {
        std::string type = val.get(0).asString();

        if (type != linInterpBuffer->getType())
        {
            LinearInterpolationBuffer * newBuffer = linInterpBuffer->cloneTo(type);

            if (newBuffer)
            {
                delete linInterpBuffer;
                linInterpBuffer = newBuffer;
            }
            else
            {
                return false;
            }
        }
    }
    else
    {
        CD_ERROR("Unsupported key: %s.\n", key.c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys)
{
    CD_DEBUG("\n");

    listOfKeys->clear();

    // Place each key in its own list so that clients can just call check('<key>') or !find('<key>').isNull().
    listOfKeys->addString("linInterpPeriodMs");
    listOfKeys->addString("linInterpBufferSize");
    listOfKeys->addString("linInterpMode");

    return true;
}

// -----------------------------------------------------------------------------
