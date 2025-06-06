/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (1.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Sun Apr 20 14:33:40 2025


#ifndef LACQUEYFETCH_PARAMSPARSER_H
#define LACQUEYFETCH_PARAMSPARSER_H

#include <yarp/os/Searchable.h>
#include <yarp/dev/IDeviceDriverParams.h>
#include <string>
#include <cmath>

/**
* This class is the parameters parser for class LacqueyFetch.
*
* These are the used parameters:
* | Group name | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
* |:----------:|:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
* | -          | canId          | int    | -     | -             | 1        | CAN bus ID  | 1-127 |
* | -          | name           | string | -     | -             | 1        | axis name   | -     |
*
* The device can be launched by yarpdev using one of the following examples (with and without all optional parameters):
* \code{.unparsed}
* yarpdev --device LacqueyFetch --canId <mandatory_value> --name <mandatory_value>
* \endcode
*
* \code{.unparsed}
* yarpdev --device LacqueyFetch --canId <mandatory_value> --name <mandatory_value>
* \endcode
*
*/

class LacqueyFetch_ParamsParser : public yarp::dev::IDeviceDriverParams
{
public:
    LacqueyFetch_ParamsParser();
    ~LacqueyFetch_ParamsParser() override = default;

public:
    const std::string m_device_classname = {"LacqueyFetch"};
    const std::string m_device_name = {"LacqueyFetch"};
    bool m_parser_is_strict = false;
    struct parser_version_type
    {
         int major = 1;
         int minor = 0;
    };
    const parser_version_type m_parser_version = {};

    const std::string m_canId_defaultValue = {""};
    const std::string m_name_defaultValue = {""};

    int m_canId = {0}; //This default value is autogenerated. It is highly recommended to provide a suggested value also for mandatory parameters.
    std::string m_name = {}; //This default value is autogenerated. It is highly recommended to provide a suggested value also for mandatory parameters.

    bool          parseParams(const yarp::os::Searchable & config) override;
    std::string   getDeviceClassName() const override { return m_device_classname; }
    std::string   getDeviceName() const override { return m_device_name; }
    std::string   getDocumentationOfDeviceParams() const override;
    std::vector<std::string> getListOfParams() const override;
};

#endif
