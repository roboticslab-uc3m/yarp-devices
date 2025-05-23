/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (1.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Sun Apr 20 14:33:40 2025


#ifndef CANBUSSOCKET_PARAMSPARSER_H
#define CANBUSSOCKET_PARAMSPARSER_H

#include <yarp/os/Searchable.h>
#include <yarp/dev/IDeviceDriverParams.h>
#include <string>
#include <cmath>

/**
* This class is the parameters parser for class CanBusSocket.
*
* These are the used parameters:
* | Group name | Parameter name      | Type        | Units | Default Value | Required | Description                                | Notes |
* |:----------:|:-------------------:|:-----------:|:-----:|:-------------:|:--------:|:------------------------------------------:|:-----:|
* | -          | port                | string      | -     | can0          | 0        | CAN socket interface                       | -     |
* | -          | bitrate             | int         | bps   | 0             | 0        | CAN bitrate                                | -     |
* | -          | blockingMode        | bool        | -     | true          | 0        | blocking mode enabled                      | -     |
* | -          | allowPermissive     | bool        | -     | false         | 0        | read/write permissive mode                 | -     |
* | -          | rxTimeoutMs         | int         | ms    | 1             | 0        | RX timeout                                 | -     |
* | -          | txTimeoutMs         | int         | ms    | 0             | 0        | TX timeout                                 | -     |
* | -          | filterFunctionCodes | bool        | -     | true          | 0        | filter mask ignores CANopen function codes | -     |
* | -          | filteredIds         | vector<int> | -     | -             | 0        | filtered node IDs                          | -     |
*
* The device can be launched by yarpdev using one of the following examples (with and without all optional parameters):
* \code{.unparsed}
* yarpdev --device CanBusSocket --port can0 --bitrate 0 --blockingMode true --allowPermissive false --rxTimeoutMs 1 --txTimeoutMs 0 --filterFunctionCodes true --filteredIds <optional_value>
* \endcode
*
* \code{.unparsed}
* yarpdev --device CanBusSocket
* \endcode
*
*/

class CanBusSocket_ParamsParser : public yarp::dev::IDeviceDriverParams
{
public:
    CanBusSocket_ParamsParser();
    ~CanBusSocket_ParamsParser() override = default;

public:
    const std::string m_device_classname = {"CanBusSocket"};
    const std::string m_device_name = {"CanBusSocket"};
    bool m_parser_is_strict = false;
    struct parser_version_type
    {
         int major = 1;
         int minor = 0;
    };
    const parser_version_type m_parser_version = {};

    const std::string m_port_defaultValue = {"can0"};
    const std::string m_bitrate_defaultValue = {"0"};
    const std::string m_blockingMode_defaultValue = {"true"};
    const std::string m_allowPermissive_defaultValue = {"false"};
    const std::string m_rxTimeoutMs_defaultValue = {"1"};
    const std::string m_txTimeoutMs_defaultValue = {"0"};
    const std::string m_filterFunctionCodes_defaultValue = {"true"};
    const std::string m_filteredIds_defaultValue = {""};

    std::string m_port = {"can0"};
    int m_bitrate = {0};
    bool m_blockingMode = {true};
    bool m_allowPermissive = {false};
    int m_rxTimeoutMs = {1};
    int m_txTimeoutMs = {0};
    bool m_filterFunctionCodes = {true};
    std::vector<int> m_filteredIds = {}; //The default value of this list is an empty list. It is highly recommended to provide a suggested value also for optional string parameters.

    bool          parseParams(const yarp::os::Searchable & config) override;
    std::string   getDeviceClassName() const override { return m_device_classname; }
    std::string   getDeviceName() const override { return m_device_name; }
    std::string   getDocumentationOfDeviceParams() const override;
    std::vector<std::string> getListOfParams() const override;
};

#endif
