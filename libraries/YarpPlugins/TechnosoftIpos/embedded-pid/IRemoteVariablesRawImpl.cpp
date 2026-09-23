// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::getRemoteVariableRaw(std::string key, yarp::os::Bottle & val)
#else
bool TechnosoftIposEmbedded::getRemoteVariableRaw(std::string key, yarp::os::Bottle & val)
#endif
{
    val.addString(key);

    if (key == "enableIp")
    {
        val.addInt32(ipBuffer ? 1 : 0);
    }
    else if (key == "ipMode")
    {
        val.addString(ipMode);
    }
    else if (key == "ipPeriodMs")
    {
        val.addInt32(ipPeriodMs);
    }
    else if (key == "enableCsv")
    {
        val.addInt32(enableCsv ? 1 : 0);
    }
    else
    {
        yCIError(IPOS, id()) << "Unsupported key:" << key;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val)
#else
bool TechnosoftIposEmbedded::setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val)
#endif
{
    if (key == "enableIp")
    {
        auto requested = val.get(0).asBool();

        if (requested ^ (ipBuffer != nullptr))
        {
            if (actualControlMode == VOCAB_CM_POSITION_DIRECT || requestedcontrolMode == VOCAB_CM_POSITION_DIRECT)
            {
                yCIError(IPOS, id()) << "Currently in posd mode, cannot change config params right now";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                return yarp::dev::ReturnValue::return_code::return_value_error_not_ready;
#else
                return false;
#endif
            }

            if (requested)
            {
                if (ipMode == "pt")
                {
                    ipBuffer = new PtBuffer(params.m_samplingPeriod, ipPeriodMs * 0.001);
                }
                else if (ipMode == "pvt")
                {
                    ipBuffer = new PvtBuffer(params.m_samplingPeriod, ipPeriodMs * 0.001);
                }

                yCIInfo(IPOS, id()) << "Created" << ipMode << "buffer with" << ipBuffer->getBufferSize()
                                    << "points and period" << ipPeriodMs << "ms";
            }
            else
            {
                delete ipBuffer;
                ipBuffer = nullptr;
                yCIInfo(IPOS, id()) << "Switched back to CSP mode";
            }
        }
    }
    else if (key == "ipMode")
    {
        if (actualControlMode == VOCAB_CM_POSITION_DIRECT || requestedcontrolMode == VOCAB_CM_POSITION_DIRECT)
        {
            yCIError(IPOS, id()) << "Currently in posd mode, cannot change ip submode right now";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue::return_code::return_value_error_not_ready;
#else
            return false;
#endif
        }

        auto value = val.get(0).asString();

        if (value == "pt" || value == "pvt")
        {
            ipMode = value;
        }
        else
        {
            yCIError(IPOS, id()) << "Illegal ip submode:" << value << "(expected 'pt' or 'pvt')";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
            return false;
#endif
        }
    }
    else if (key == "ipPeriodMs")
    {
        if (actualControlMode == VOCAB_CM_POSITION_DIRECT || requestedcontrolMode == VOCAB_CM_POSITION_DIRECT)
        {
            yCIError(IPOS, id()) << "Currently in posd mode, cannot change ip period right now";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue::return_code::return_value_error_not_ready;
#else
            return false;
#endif
        }

        auto value = val.get(0).asInt32();

        if (value >= 0)
        {
            ipPeriodMs = value;
        }
        else
        {
            yCIError(IPOS, id()) << "Illegal ip period:" << value;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
            return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
            return false;
#endif
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
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
                return yarp::dev::ReturnValue::return_code::return_value_error_not_ready;
#else
                return false;
#endif
            }

            enableCsv = requested;
            yCIInfo(IPOS, id()) << "CSV mode" << (requested ? "enabled" : "disabled");
        }
    }
    else
    {
        yCIError(IPOS, id()) << "Unsupported key:" << key;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposEmbedded::getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys)
#else
bool TechnosoftIposEmbedded::getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys)
#endif
{
    listOfKeys->clear();

    // the order is relevant, e.g. enableIp depends on the ip* variables
    listOfKeys->addString("ipMode");
    listOfKeys->addString("ipPeriodMs");
    listOfKeys->addString("enableIp");
    listOfKeys->addString("enableCsv");

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
