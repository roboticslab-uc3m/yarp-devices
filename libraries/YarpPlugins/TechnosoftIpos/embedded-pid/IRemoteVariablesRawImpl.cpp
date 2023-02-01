// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRemoteVariableRaw(std::string key, yarp::os::Bottle & val)
{
    yCITrace(IPOS, id(), "%s: %s", key.c_str(), val.toString().c_str());

    if (key == "enableIp")
    {
        val.addInt32(ipBuffer ? 1 : 0);
    }
    else if (key == "ipMode")
    {
        val.addString(ipBuffer ? ipBuffer->getType() : ipMode);
    }
    else if (key == "ipPeriodMs")
    {
        val.addInt32(ipBuffer ? ipBuffer->getPeriodMs() : ipPeriodMs);
    }
    else if (key == "enableCsv")
    {
        val.addInt32(enableCsv ? 1 : 0);
    }
    else
    {
        val.addInt32(0);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val)
{
    yCITrace(IPOS, id(), "%s", key.c_str());

    if (key == "enableIp")
    {
        auto requested = val.get(0).asBool();

        if (requested ^ (ipBuffer != nullptr))
        {
            if (actualControlMode == VOCAB_CM_POSITION_DIRECT || requestedcontrolMode == VOCAB_CM_POSITION_DIRECT)
            {
                yCIError(IPOS, id()) << "Currently in posd mode, cannot change config params right now";
                return false;
            }

            if (requested)
            {
                if (ipMode == "pt")
                {
                    ipBuffer = new PtBuffer(samplingPeriod, ipPeriodMs * 0.001);
                }
                else if (ipMode == "pvt")
                {
                    ipBuffer = new PvtBuffer(samplingPeriod, ipPeriodMs * 0.001);
                }

                yCIInfo(IPOS, id()) << "Created" << ipBuffer->getType() << "buffer with" << ipBuffer->getBufferSize()
                                    << "points and period" << ipBuffer->getPeriodMs() << "ms";
            }
            else
            {
                delete ipBuffer;
                ipBuffer = nullptr;
                yCIInfo(IPOS, id()) << "Switched back to CSP mode";
            }
        }
        else
        {
            yCIWarning(IPOS, id()) << "IP mode already" << (requested ? "enabled" : "disabled");
        }
    }
    else if (key == "ipMode")
    {
        if (actualControlMode == VOCAB_CM_POSITION_DIRECT || requestedcontrolMode == VOCAB_CM_POSITION_DIRECT)
        {
            yCIError(IPOS, id()) << "Currently in posd mode, cannot change ip submode right now";
            return false;
        }

        auto value = val.get(0).asString();

        if (value == "pt" || value == "pvt")
        {
            ipMode = value;
        }
        else
        {
            yCIError(IPOS, id()) << "Illegal ip submode:" << value;
            return false;
        }
    }
    else if (key == "ipPeriodMs")
    {
        if (actualControlMode == VOCAB_CM_POSITION_DIRECT || requestedcontrolMode == VOCAB_CM_POSITION_DIRECT)
        {
            yCIError(IPOS, id()) << "Currently in posd mode, cannot change ip period right now";
            return false;
        }

        auto value = val.get(0).asInt32();

        if (value >= 0)
        {
            ipPeriodMs = value;
        }
        else
        {
            yCIError(IPOS, id()) << "Illegal ip period:" << value;
            return false;
        }
    }
    else if (key == "enableCsv")
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
        }
        else
        {
            yCIWarning(IPOS, id()) << "CSV mode already" << (requested ? "enabled" : "disabled");
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys)
{
    yCITrace(IPOS, id());

    listOfKeys->clear();
    listOfKeys->addString("enableIp");
    listOfKeys->addString("ipMode");
    listOfKeys->addString("ipPeriodMs");
    listOfKeys->addString("enableCsv");

    return true;
}

// -----------------------------------------------------------------------------
