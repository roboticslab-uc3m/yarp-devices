/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (1.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Sun Apr 20 16:50:20 2025


#include "TechnosoftIpos_ParamsParser.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

namespace {
    YARP_LOG_COMPONENT(TechnosoftIposParamsCOMPONENT, "yarp.device.TechnosoftIpos")
}


TechnosoftIpos_ParamsParser::TechnosoftIpos_ParamsParser()
{
}


std::vector<std::string> TechnosoftIpos_ParamsParser::getListOfParams() const
{
    std::vector<std::string> params;
    params.push_back("canId");
    params.push_back("useEmbeddedPid");
    params.push_back("name");
    params.push_back("type");
    params.push_back("max");
    params.push_back("min");
    params.push_back("maxVel");
    params.push_back("refSpeed");
    params.push_back("refAcceleration");
    params.push_back("extraTr");
    params.push_back("samplingPeriod");
    params.push_back("reverse");
    params.push_back("heartbeatPeriod");
    params.push_back("syncPeriod");
    params.push_back("initialControlMode");
    params.push_back("sdoTimeout");
    params.push_back("driveStateTimeout");
    params.push_back("tpdo1InhibitTime");
    params.push_back("tpdo1EventTimer");
    params.push_back("tpdo2InhibitTime");
    params.push_back("tpdo2EventTimer");
    params.push_back("monitorPeriod");
    params.push_back("driver::peakCurrent");
    params.push_back("motor::k");
    params.push_back("gearbox::tr");
    params.push_back("encoder::encoderPulses");
    params.push_back("ipMode");
    params.push_back("ipPeriodMs");
    params.push_back("enableIp");
    params.push_back("enableCsv");
    params.push_back("initialInteractionMode");
    params.push_back("stiffness");
    params.push_back("damping");
    params.push_back("impedanceOffset");
    params.push_back("minStiffness");
    params.push_back("maxStiffness");
    params.push_back("minDamping");
    params.push_back("maxDamping");
    params.push_back("kp");
    params.push_back("ki");
    params.push_back("kd");
    params.push_back("maxInt");
    params.push_back("maxOutput");
    params.push_back("offset");
    params.push_back("scale");
    params.push_back("stictionUp");
    params.push_back("stictionDown");
    params.push_back("kff");
    params.push_back("errorLimit");
    return params;
}


bool      TechnosoftIpos_ParamsParser::parseParams(const yarp::os::Searchable & config)
{
    //Check for --help option
    if (config.check("help"))
    {
        yCInfo(TechnosoftIposParamsCOMPONENT) << getDocumentationOfDeviceParams();
    }

    std::string config_string = config.toString();
    yarp::os::Property prop_check(config_string.c_str());
    //Parser of parameter canId
    {
        if (config.check("canId"))
        {
            m_canId = config.find("canId").asInt64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'canId' using value:" << m_canId;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'canId' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: CAN bus ID";
            return false;
        }
        prop_check.unput("canId");
    }

    //Parser of parameter useEmbeddedPid
    {
        if (config.check("useEmbeddedPid"))
        {
            m_useEmbeddedPid = config.find("useEmbeddedPid").asBool();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'useEmbeddedPid' using value:" << m_useEmbeddedPid;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'useEmbeddedPid' using DEFAULT value:" << m_useEmbeddedPid;
        }
        prop_check.unput("useEmbeddedPid");
    }

    //Parser of parameter name
    {
        if (config.check("name"))
        {
            m_name = config.find("name").asString();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'name' using value:" << m_name;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'name' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: axis name";
            return false;
        }
        prop_check.unput("name");
    }

    //Parser of parameter type
    {
        if (config.check("type"))
        {
            m_type = config.find("type").asString();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'type' using value:" << m_type;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'type' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: joint type vocab";
            return false;
        }
        prop_check.unput("type");
    }

    //Parser of parameter max
    {
        if (config.check("max"))
        {
            m_max = config.find("max").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'max' using value:" << m_max;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'max' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: upper joint limit";
            yCError(TechnosoftIposParamsCOMPONENT) << "Remember: Units for this parameter are: 'deg or m'";
            return false;
        }
        prop_check.unput("max");
    }

    //Parser of parameter min
    {
        if (config.check("min"))
        {
            m_min = config.find("min").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'min' using value:" << m_min;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'min' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: lower joint limit";
            yCError(TechnosoftIposParamsCOMPONENT) << "Remember: Units for this parameter are: 'deg or m'";
            return false;
        }
        prop_check.unput("min");
    }

    //Parser of parameter maxVel
    {
        if (config.check("maxVel"))
        {
            m_maxVel = config.find("maxVel").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'maxVel' using value:" << m_maxVel;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'maxVel' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: maximum velocity";
            yCError(TechnosoftIposParamsCOMPONENT) << "Remember: Units for this parameter are: 'deg/s or m/s'";
            return false;
        }
        prop_check.unput("maxVel");
    }

    //Parser of parameter refSpeed
    {
        if (config.check("refSpeed"))
        {
            m_refSpeed = config.find("refSpeed").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'refSpeed' using value:" << m_refSpeed;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'refSpeed' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: reference speed";
            yCError(TechnosoftIposParamsCOMPONENT) << "Remember: Units for this parameter are: 'deg/s or m/s'";
            return false;
        }
        prop_check.unput("refSpeed");
    }

    //Parser of parameter refAcceleration
    {
        if (config.check("refAcceleration"))
        {
            m_refAcceleration = config.find("refAcceleration").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'refAcceleration' using value:" << m_refAcceleration;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'refAcceleration' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: reference acceleration";
            yCError(TechnosoftIposParamsCOMPONENT) << "Remember: Units for this parameter are: 'deg/s^2 or m/s^2'";
            return false;
        }
        prop_check.unput("refAcceleration");
    }

    //Parser of parameter extraTr
    {
        if (config.check("extraTr"))
        {
            m_extraTr = config.find("extraTr").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'extraTr' using value:" << m_extraTr;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'extraTr' using DEFAULT value:" << m_extraTr;
        }
        prop_check.unput("extraTr");
    }

    //Parser of parameter samplingPeriod
    {
        if (config.check("samplingPeriod"))
        {
            m_samplingPeriod = config.find("samplingPeriod").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'samplingPeriod' using value:" << m_samplingPeriod;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'samplingPeriod' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: controller sampling period";
            yCError(TechnosoftIposParamsCOMPONENT) << "Remember: Units for this parameter are: 's'";
            return false;
        }
        prop_check.unput("samplingPeriod");
    }

    //Parser of parameter reverse
    {
        if (config.check("reverse"))
        {
            m_reverse = config.find("reverse").asBool();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'reverse' using value:" << m_reverse;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'reverse' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: reverse motor encoder counts";
            return false;
        }
        prop_check.unput("reverse");
    }

    //Parser of parameter heartbeatPeriod
    {
        if (config.check("heartbeatPeriod"))
        {
            m_heartbeatPeriod = config.find("heartbeatPeriod").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'heartbeatPeriod' using value:" << m_heartbeatPeriod;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'heartbeatPeriod' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: CAN heartbeat period";
            yCError(TechnosoftIposParamsCOMPONENT) << "Remember: Units for this parameter are: 's'";
            return false;
        }
        prop_check.unput("heartbeatPeriod");
    }

    //Parser of parameter syncPeriod
    {
        if (config.check("syncPeriod"))
        {
            m_syncPeriod = config.find("syncPeriod").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'syncPeriod' using value:" << m_syncPeriod;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'syncPeriod' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: CAN SYNC message period";
            yCError(TechnosoftIposParamsCOMPONENT) << "Remember: Units for this parameter are: 's'";
            return false;
        }
        prop_check.unput("syncPeriod");
    }

    //Parser of parameter initialControlMode
    {
        if (config.check("initialControlMode"))
        {
            m_initialControlMode = config.find("initialControlMode").asString();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'initialControlMode' using value:" << m_initialControlMode;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'initialControlMode' using DEFAULT value:" << m_initialControlMode;
        }
        prop_check.unput("initialControlMode");
    }

    //Parser of parameter sdoTimeout
    {
        if (config.check("sdoTimeout"))
        {
            m_sdoTimeout = config.find("sdoTimeout").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'sdoTimeout' using value:" << m_sdoTimeout;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'sdoTimeout' using DEFAULT value:" << m_sdoTimeout;
        }
        prop_check.unput("sdoTimeout");
    }

    //Parser of parameter driveStateTimeout
    {
        if (config.check("driveStateTimeout"))
        {
            m_driveStateTimeout = config.find("driveStateTimeout").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'driveStateTimeout' using value:" << m_driveStateTimeout;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'driveStateTimeout' using DEFAULT value:" << m_driveStateTimeout;
        }
        prop_check.unput("driveStateTimeout");
    }

    //Parser of parameter tpdo1InhibitTime
    {
        if (config.check("tpdo1InhibitTime"))
        {
            m_tpdo1InhibitTime = config.find("tpdo1InhibitTime").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'tpdo1InhibitTime' using value:" << m_tpdo1InhibitTime;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'tpdo1InhibitTime' using DEFAULT value:" << m_tpdo1InhibitTime;
        }
        prop_check.unput("tpdo1InhibitTime");
    }

    //Parser of parameter tpdo1EventTimer
    {
        if (config.check("tpdo1EventTimer"))
        {
            m_tpdo1EventTimer = config.find("tpdo1EventTimer").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'tpdo1EventTimer' using value:" << m_tpdo1EventTimer;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'tpdo1EventTimer' using DEFAULT value:" << m_tpdo1EventTimer;
        }
        prop_check.unput("tpdo1EventTimer");
    }

    //Parser of parameter tpdo2InhibitTime
    {
        if (config.check("tpdo2InhibitTime"))
        {
            m_tpdo2InhibitTime = config.find("tpdo2InhibitTime").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'tpdo2InhibitTime' using value:" << m_tpdo2InhibitTime;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'tpdo2InhibitTime' using DEFAULT value:" << m_tpdo2InhibitTime;
        }
        prop_check.unput("tpdo2InhibitTime");
    }

    //Parser of parameter tpdo2EventTimer
    {
        if (config.check("tpdo2EventTimer"))
        {
            m_tpdo2EventTimer = config.find("tpdo2EventTimer").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'tpdo2EventTimer' using value:" << m_tpdo2EventTimer;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'tpdo2EventTimer' using DEFAULT value:" << m_tpdo2EventTimer;
        }
        prop_check.unput("tpdo2EventTimer");
    }

    //Parser of parameter monitorPeriod
    {
        if (config.check("monitorPeriod"))
        {
            m_monitorPeriod = config.find("monitorPeriod").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'monitorPeriod' using value:" << m_monitorPeriod;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'monitorPeriod' using DEFAULT value:" << m_monitorPeriod;
        }
        prop_check.unput("monitorPeriod");
    }

    //Parser of parameter driver::peakCurrent
    {
        yarp::os::Bottle sectionp;
        sectionp = config.findGroup("driver");
        if (sectionp.check("peakCurrent"))
        {
            m_driver_peakCurrent = sectionp.find("peakCurrent").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'driver::peakCurrent' using value:" << m_driver_peakCurrent;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'driver::peakCurrent' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: drive peak current";
            yCError(TechnosoftIposParamsCOMPONENT) << "Remember: Units for this parameter are: 'A'";
            return false;
        }
        prop_check.unput("driver::peakCurrent");
    }

    //Parser of parameter motor::k
    {
        yarp::os::Bottle sectionp;
        sectionp = config.findGroup("motor");
        if (sectionp.check("k"))
        {
            m_motor_k = sectionp.find("k").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'motor::k' using value:" << m_motor_k;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'motor::k' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: motor constant";
            yCError(TechnosoftIposParamsCOMPONENT) << "Remember: Units for this parameter are: 'N*m/A'";
            return false;
        }
        prop_check.unput("motor::k");
    }

    //Parser of parameter gearbox::tr
    {
        yarp::os::Bottle sectionp;
        sectionp = config.findGroup("gearbox");
        if (sectionp.check("tr"))
        {
            m_gearbox_tr = sectionp.find("tr").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'gearbox::tr' using value:" << m_gearbox_tr;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'gearbox::tr' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: reduction";
            return false;
        }
        prop_check.unput("gearbox::tr");
    }

    //Parser of parameter encoder::encoderPulses
    {
        yarp::os::Bottle sectionp;
        sectionp = config.findGroup("encoder");
        if (sectionp.check("encoderPulses"))
        {
            m_encoder_encoderPulses = sectionp.find("encoderPulses").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'encoder::encoderPulses' using value:" << m_encoder_encoderPulses;
        }
        else
        {
            yCError(TechnosoftIposParamsCOMPONENT) << "Mandatory parameter 'encoder::encoderPulses' not found!";
            yCError(TechnosoftIposParamsCOMPONENT) << "Description of the parameter: encoder pulses per revolution";
            return false;
        }
        prop_check.unput("encoder::encoderPulses");
    }

    //Parser of parameter ipMode
    {
        if (config.check("ipMode"))
        {
            m_ipMode = config.find("ipMode").asString();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'ipMode' using value:" << m_ipMode;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'ipMode' using DEFAULT value:" << m_ipMode;
        }
        prop_check.unput("ipMode");
    }

    //Parser of parameter ipPeriodMs
    {
        if (config.check("ipPeriodMs"))
        {
            m_ipPeriodMs = config.find("ipPeriodMs").asInt64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'ipPeriodMs' using value:" << m_ipPeriodMs;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'ipPeriodMs' using DEFAULT value:" << m_ipPeriodMs;
        }
        prop_check.unput("ipPeriodMs");
    }

    //Parser of parameter enableIp
    {
        if (config.check("enableIp"))
        {
            m_enableIp = config.find("enableIp").asBool();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'enableIp' using value:" << m_enableIp;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'enableIp' using DEFAULT value:" << m_enableIp;
        }
        prop_check.unput("enableIp");
    }

    //Parser of parameter enableCsv
    {
        if (config.check("enableCsv"))
        {
            m_enableCsv = config.find("enableCsv").asBool();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'enableCsv' using value:" << m_enableCsv;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'enableCsv' using DEFAULT value:" << m_enableCsv;
        }
        prop_check.unput("enableCsv");
    }

    //Parser of parameter initialInteractionMode
    {
        if (config.check("initialInteractionMode"))
        {
            m_initialInteractionMode = config.find("initialInteractionMode").asString();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'initialInteractionMode' using value:" << m_initialInteractionMode;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'initialInteractionMode' using DEFAULT value:" << m_initialInteractionMode;
        }
        prop_check.unput("initialInteractionMode");
    }

    //Parser of parameter stiffness
    {
        if (config.check("stiffness"))
        {
            m_stiffness = config.find("stiffness").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'stiffness' using value:" << m_stiffness;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'stiffness' using DEFAULT value:" << m_stiffness;
        }
        prop_check.unput("stiffness");
    }

    //Parser of parameter damping
    {
        if (config.check("damping"))
        {
            m_damping = config.find("damping").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'damping' using value:" << m_damping;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'damping' using DEFAULT value:" << m_damping;
        }
        prop_check.unput("damping");
    }

    //Parser of parameter impedanceOffset
    {
        if (config.check("impedanceOffset"))
        {
            m_impedanceOffset = config.find("impedanceOffset").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'impedanceOffset' using value:" << m_impedanceOffset;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'impedanceOffset' using DEFAULT value:" << m_impedanceOffset;
        }
        prop_check.unput("impedanceOffset");
    }

    //Parser of parameter minStiffness
    {
        if (config.check("minStiffness"))
        {
            m_minStiffness = config.find("minStiffness").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'minStiffness' using value:" << m_minStiffness;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'minStiffness' using DEFAULT value:" << m_minStiffness;
        }
        prop_check.unput("minStiffness");
    }

    //Parser of parameter maxStiffness
    {
        if (config.check("maxStiffness"))
        {
            m_maxStiffness = config.find("maxStiffness").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'maxStiffness' using value:" << m_maxStiffness;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'maxStiffness' using DEFAULT value:" << m_maxStiffness;
        }
        prop_check.unput("maxStiffness");
    }

    //Parser of parameter minDamping
    {
        if (config.check("minDamping"))
        {
            m_minDamping = config.find("minDamping").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'minDamping' using value:" << m_minDamping;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'minDamping' using DEFAULT value:" << m_minDamping;
        }
        prop_check.unput("minDamping");
    }

    //Parser of parameter maxDamping
    {
        if (config.check("maxDamping"))
        {
            m_maxDamping = config.find("maxDamping").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'maxDamping' using value:" << m_maxDamping;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'maxDamping' using DEFAULT value:" << m_maxDamping;
        }
        prop_check.unput("maxDamping");
    }

    //Parser of parameter kp
    {
        if (config.check("kp"))
        {
            m_kp = config.find("kp").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'kp' using value:" << m_kp;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'kp' using DEFAULT value:" << m_kp;
        }
        prop_check.unput("kp");
    }

    //Parser of parameter ki
    {
        if (config.check("ki"))
        {
            m_ki = config.find("ki").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'ki' using value:" << m_ki;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'ki' using DEFAULT value:" << m_ki;
        }
        prop_check.unput("ki");
    }

    //Parser of parameter kd
    {
        if (config.check("kd"))
        {
            m_kd = config.find("kd").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'kd' using value:" << m_kd;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'kd' using DEFAULT value:" << m_kd;
        }
        prop_check.unput("kd");
    }

    //Parser of parameter maxInt
    {
        if (config.check("maxInt"))
        {
            m_maxInt = config.find("maxInt").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'maxInt' using value:" << m_maxInt;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'maxInt' using DEFAULT value:" << m_maxInt;
        }
        prop_check.unput("maxInt");
    }

    //Parser of parameter maxOutput
    {
        if (config.check("maxOutput"))
        {
            m_maxOutput = config.find("maxOutput").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'maxOutput' using value:" << m_maxOutput;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'maxOutput' using DEFAULT value:" << m_maxOutput;
        }
        prop_check.unput("maxOutput");
    }

    //Parser of parameter offset
    {
        if (config.check("offset"))
        {
            m_offset = config.find("offset").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'offset' using value:" << m_offset;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'offset' using DEFAULT value:" << m_offset;
        }
        prop_check.unput("offset");
    }

    //Parser of parameter scale
    {
        if (config.check("scale"))
        {
            m_scale = config.find("scale").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'scale' using value:" << m_scale;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'scale' using DEFAULT value:" << m_scale;
        }
        prop_check.unput("scale");
    }

    //Parser of parameter stictionUp
    {
        if (config.check("stictionUp"))
        {
            m_stictionUp = config.find("stictionUp").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'stictionUp' using value:" << m_stictionUp;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'stictionUp' using DEFAULT value:" << m_stictionUp;
        }
        prop_check.unput("stictionUp");
    }

    //Parser of parameter stictionDown
    {
        if (config.check("stictionDown"))
        {
            m_stictionDown = config.find("stictionDown").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'stictionDown' using value:" << m_stictionDown;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'stictionDown' using DEFAULT value:" << m_stictionDown;
        }
        prop_check.unput("stictionDown");
    }

    //Parser of parameter kff
    {
        if (config.check("kff"))
        {
            m_kff = config.find("kff").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'kff' using value:" << m_kff;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'kff' using DEFAULT value:" << m_kff;
        }
        prop_check.unput("kff");
    }

    //Parser of parameter errorLimit
    {
        if (config.check("errorLimit"))
        {
            m_errorLimit = config.find("errorLimit").asFloat64();
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'errorLimit' using value:" << m_errorLimit;
        }
        else
        {
            yCInfo(TechnosoftIposParamsCOMPONENT) << "Parameter 'errorLimit' using DEFAULT value:" << m_errorLimit;
        }
        prop_check.unput("errorLimit");
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
                yCError(TechnosoftIposParamsCOMPONENT) << "User asking for parameter: "<<it->name <<" which is unknown to this parser!";
                extra_params_found = true;
            }
            else
            {
                yCWarning(TechnosoftIposParamsCOMPONENT) << "User asking for parameter: "<< it->name <<" which is unknown to this parser!";
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


std::string      TechnosoftIpos_ParamsParser::getDocumentationOfDeviceParams() const
{
    std::string doc;
    doc = doc + std::string("\n=============================================\n");
    doc = doc + std::string("This is the help for device: TechnosoftIpos\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("This is the list of the parameters accepted by the device:\n");
    doc = doc + std::string("'canId': CAN bus ID\n");
    doc = doc + std::string("'useEmbeddedPid': use embedded PID\n");
    doc = doc + std::string("'name': axis name\n");
    doc = doc + std::string("'type': joint type vocab\n");
    doc = doc + std::string("'max': upper joint limit\n");
    doc = doc + std::string("'min': lower joint limit\n");
    doc = doc + std::string("'maxVel': maximum velocity\n");
    doc = doc + std::string("'refSpeed': reference speed\n");
    doc = doc + std::string("'refAcceleration': reference acceleration\n");
    doc = doc + std::string("'extraTr': additional reduction\n");
    doc = doc + std::string("'samplingPeriod': controller sampling period\n");
    doc = doc + std::string("'reverse': reverse motor encoder counts\n");
    doc = doc + std::string("'heartbeatPeriod': CAN heartbeat period\n");
    doc = doc + std::string("'syncPeriod': CAN SYNC message period\n");
    doc = doc + std::string("'initialControlMode': initial control mode vocab\n");
    doc = doc + std::string("'sdoTimeout': CAN SDO timeout\n");
    doc = doc + std::string("'driveStateTimeout': CAN drive state timeout\n");
    doc = doc + std::string("'tpdo1InhibitTime': CAN TPDO1 inhibit time\n");
    doc = doc + std::string("'tpdo1EventTimer': CAN TPDO1 event timer\n");
    doc = doc + std::string("'tpdo2InhibitTime': CAN TPDO2 inhibit time\n");
    doc = doc + std::string("'tpdo2EventTimer': CAN TPDO2 event timer\n");
    doc = doc + std::string("'monitorPeriod': monitor thread period\n");
    doc = doc + std::string("'driver::peakCurrent': drive peak current\n");
    doc = doc + std::string("'motor::k': motor constant\n");
    doc = doc + std::string("'gearbox::tr': reduction\n");
    doc = doc + std::string("'encoder::encoderPulses': encoder pulses per revolution\n");
    doc = doc + std::string("'ipMode': IP mode (pt, pvt)\n");
    doc = doc + std::string("'ipPeriodMs': IP period\n");
    doc = doc + std::string("'enableIp': enable IP mode\n");
    doc = doc + std::string("'enableCsv': enable CSV mode\n");
    doc = doc + std::string("'initialInteractionMode': initial interaction mode vocab\n");
    doc = doc + std::string("'stiffness': impedance stiffness\n");
    doc = doc + std::string("'damping': impedance damping\n");
    doc = doc + std::string("'impedanceOffset': impedance offset\n");
    doc = doc + std::string("'minStiffness': minimum impedance stiffness\n");
    doc = doc + std::string("'maxStiffness': maximum impedance stiffness\n");
    doc = doc + std::string("'minDamping': minimum impedance damping\n");
    doc = doc + std::string("'maxDamping': maximum impedance damping\n");
    doc = doc + std::string("'kp': position PID Kp\n");
    doc = doc + std::string("'ki': position PID Ki\n");
    doc = doc + std::string("'kd': position PID Kd\n");
    doc = doc + std::string("'maxInt': position PID saturation threshold\n");
    doc = doc + std::string("'maxOutput': position PID maximum output\n");
    doc = doc + std::string("'offset': position PID offset\n");
    doc = doc + std::string("'scale': position PID scale\n");
    doc = doc + std::string("'stictionUp': position PID stiction up\n");
    doc = doc + std::string("'stictionDown': position PID stiction down\n");
    doc = doc + std::string("'kff': position PID feed-forward\n");
    doc = doc + std::string("'errorLimit': position PID error limit\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("Here are some examples of invocation command with yarpdev, with all params:\n");
    doc = doc + " yarpdev --device TechnosoftIpos --canId <mandatory_value> --useEmbeddedPid true --name <mandatory_value> --type <mandatory_value> --max <mandatory_value> --min <mandatory_value> --maxVel <mandatory_value> --refSpeed <mandatory_value> --refAcceleration <mandatory_value> --extraTr 1.0 --samplingPeriod <mandatory_value> --reverse <mandatory_value> --heartbeatPeriod <mandatory_value> --syncPeriod <mandatory_value> --initialControlMode idl --sdoTimeout 0.02 --driveStateTimeout 2.0 --tpdo1InhibitTime 0.0 --tpdo1EventTimer 0.0 --tpdo2InhibitTime 0.0 --tpdo2EventTimer 0.0 --monitorPeriod 0.0 --driver::peakCurrent <mandatory_value> --motor::k <mandatory_value> --gearbox::tr <mandatory_value> --encoder::encoderPulses <mandatory_value> --ipMode pt --ipPeriodMs 50 --enableIp false --enableCsv false --initialInteractionMode unkn --stiffness 0.0 --damping 0.0 --impedanceOffset 0.0 --minStiffness 0.0 --maxStiffness 0.0 --minDamping 0.0 --maxDamping 0.0 --kp 0.0 --ki 0.0 --kd 0.0 --maxInt 0.0 --maxOutput 0.0 --offset 0.0 --scale 1.0 --stictionUp 0.0 --stictionDown 0.0 --kff 0.0 --errorLimit 0.0\n";
    doc = doc + std::string("Using only mandatory params:\n");
    doc = doc + " yarpdev --device TechnosoftIpos --canId <mandatory_value> --name <mandatory_value> --type <mandatory_value> --max <mandatory_value> --min <mandatory_value> --maxVel <mandatory_value> --refSpeed <mandatory_value> --refAcceleration <mandatory_value> --samplingPeriod <mandatory_value> --reverse <mandatory_value> --heartbeatPeriod <mandatory_value> --syncPeriod <mandatory_value> --driver::peakCurrent <mandatory_value> --motor::k <mandatory_value> --gearbox::tr <mandatory_value> --encoder::encoderPulses <mandatory_value>\n";
    doc = doc + std::string("=============================================\n\n");    return doc;
}
