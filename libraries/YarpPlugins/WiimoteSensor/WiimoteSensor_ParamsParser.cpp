/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (1.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Sun Apr 20 14:33:40 2025


#include "WiimoteSensor_ParamsParser.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

namespace {
    YARP_LOG_COMPONENT(WiimoteSensorParamsCOMPONENT, "yarp.device.WiimoteSensor")
}


WiimoteSensor_ParamsParser::WiimoteSensor_ParamsParser()
{
}


std::vector<std::string> WiimoteSensor_ParamsParser::getListOfParams() const
{
    std::vector<std::string> params;
    params.push_back("deviceId");
    params.push_back("calibZeroX");
    params.push_back("calibZeroY");
    params.push_back("calibZeroZ");
    params.push_back("calibOneX");
    params.push_back("calibOneY");
    params.push_back("calibOneZ");
    return params;
}


bool      WiimoteSensor_ParamsParser::parseParams(const yarp::os::Searchable & config)
{
    //Check for --help option
    if (config.check("help"))
    {
        yCInfo(WiimoteSensorParamsCOMPONENT) << getDocumentationOfDeviceParams();
    }

    std::string config_string = config.toString();
    yarp::os::Property prop_check(config_string.c_str());
    //Parser of parameter deviceId
    {
        if (config.check("deviceId"))
        {
            m_deviceId = config.find("deviceId").asInt64();
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'deviceId' using value:" << m_deviceId;
        }
        else
        {
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'deviceId' using DEFAULT value:" << m_deviceId;
        }
        prop_check.unput("deviceId");
    }

    //Parser of parameter calibZeroX
    {
        if (config.check("calibZeroX"))
        {
            m_calibZeroX = config.find("calibZeroX").asInt64();
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'calibZeroX' using value:" << m_calibZeroX;
        }
        else
        {
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'calibZeroX' using DEFAULT value:" << m_calibZeroX;
        }
        prop_check.unput("calibZeroX");
    }

    //Parser of parameter calibZeroY
    {
        if (config.check("calibZeroY"))
        {
            m_calibZeroY = config.find("calibZeroY").asInt64();
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'calibZeroY' using value:" << m_calibZeroY;
        }
        else
        {
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'calibZeroY' using DEFAULT value:" << m_calibZeroY;
        }
        prop_check.unput("calibZeroY");
    }

    //Parser of parameter calibZeroZ
    {
        if (config.check("calibZeroZ"))
        {
            m_calibZeroZ = config.find("calibZeroZ").asInt64();
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'calibZeroZ' using value:" << m_calibZeroZ;
        }
        else
        {
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'calibZeroZ' using DEFAULT value:" << m_calibZeroZ;
        }
        prop_check.unput("calibZeroZ");
    }

    //Parser of parameter calibOneX
    {
        if (config.check("calibOneX"))
        {
            m_calibOneX = config.find("calibOneX").asInt64();
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'calibOneX' using value:" << m_calibOneX;
        }
        else
        {
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'calibOneX' using DEFAULT value:" << m_calibOneX;
        }
        prop_check.unput("calibOneX");
    }

    //Parser of parameter calibOneY
    {
        if (config.check("calibOneY"))
        {
            m_calibOneY = config.find("calibOneY").asInt64();
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'calibOneY' using value:" << m_calibOneY;
        }
        else
        {
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'calibOneY' using DEFAULT value:" << m_calibOneY;
        }
        prop_check.unput("calibOneY");
    }

    //Parser of parameter calibOneZ
    {
        if (config.check("calibOneZ"))
        {
            m_calibOneZ = config.find("calibOneZ").asInt64();
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'calibOneZ' using value:" << m_calibOneZ;
        }
        else
        {
            yCInfo(WiimoteSensorParamsCOMPONENT) << "Parameter 'calibOneZ' using DEFAULT value:" << m_calibOneZ;
        }
        prop_check.unput("calibOneZ");
    }

    /*
    //This code check if the user set some parameter which are not check by the parser
    //If the parser is set in strict mode, this will generate an error
    if (prop_check.size() > 0)
    {
        bool extra_params_found = false;
        for (auto it=prop_check.begin(); it!=prop_check.end(); it++)
        {
            if (m_parser_is_strict)
            {
                yCError(WiimoteSensorParamsCOMPONENT) << "User asking for parameter: "<<it->name <<" which is unknown to this parser!";
                extra_params_found = true;
            }
            else
            {
                yCWarning(WiimoteSensorParamsCOMPONENT) << "User asking for parameter: "<< it->name <<" which is unknown to this parser!";
            }
        }

       if (m_parser_is_strict && extra_params_found)
       {
           return false;
       }
    }
    */
    return true;
}


std::string      WiimoteSensor_ParamsParser::getDocumentationOfDeviceParams() const
{
    std::string doc;
    doc = doc + std::string("\n=============================================\n");
    doc = doc + std::string("This is the help for device: WiimoteSensor\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("This is the list of the parameters accepted by the device:\n");
    doc = doc + std::string("'deviceId': Wiimote device number\n");
    doc = doc + std::string("'calibZeroX': normalization value for X axis (zero)\n");
    doc = doc + std::string("'calibZeroY': normalization value for Y axis (zero)\n");
    doc = doc + std::string("'calibZeroZ': normalization value for Z axis (zero)\n");
    doc = doc + std::string("'calibOneX': normalization value for X axis (one)\n");
    doc = doc + std::string("'calibOneY': normalization value for Y axis (one)\n");
    doc = doc + std::string("'calibOneZ': normalization value for Z axis (one)\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("Here are some examples of invocation command with yarpdev, with all params:\n");
    doc = doc + " yarpdev --device WiimoteSensor --deviceId 1 --calibZeroX -30 --calibZeroY -22 --calibZeroZ 72 --calibOneX 69 --calibOneY -123 --calibOneZ -25\n";
    doc = doc + std::string("Using only mandatory params:\n");
    doc = doc + " yarpdev --device WiimoteSensor\n";
    doc = doc + std::string("=============================================\n\n");    return doc;
}
