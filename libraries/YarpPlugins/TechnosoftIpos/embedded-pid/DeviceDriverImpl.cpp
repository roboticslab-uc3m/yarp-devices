// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_IP_MODE = "pt";
constexpr auto DEFAULT_IP_PERIOD_MS = 50; // ms
constexpr auto DEFAULT_ENABLE_IP = false;
constexpr auto DEFAULT_ENABLE_CSV = false;

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::open(yarp::os::Searchable & config)
{
    yarp::os::Property options;
    options.fromString(config.findGroup("common").toString());
    options.fromString(config.toString(), false); // override common options

    auto ipModeVal = options.check("ipMode", yarp::os::Value(DEFAULT_IP_MODE), "IP mode (pt, pvt)");
    auto ipPeriodMsVal = options.check("ipPeriodMs", yarp::os::Value(DEFAULT_IP_PERIOD_MS), "IP period (ms)");
    auto enableIpVal = options.check("enableIp", yarp::os::Value(DEFAULT_ENABLE_IP), "enable IP mode");
    auto enableCsvVal = options.check("enableCsv", yarp::os::Value(DEFAULT_ENABLE_CSV), "enable CSV mode");

    if (!setRemoteVariableRaw("ipMode", {ipModeVal}) || !setRemoteVariableRaw("ipPeriodMs", {ipPeriodMsVal}) ||
        !setRemoteVariableRaw("enableIp", {enableIpVal}) || !setRemoteVariableRaw("enableCsv", {enableCsvVal}))
    {
        return false;
    }

    return TechnosoftIposBase::open(config);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::close()
{
    if (ipBuffer)
    {
        delete ipBuffer;
        ipBuffer = nullptr;
    }

    return TechnosoftIposBase::close();
}

// -----------------------------------------------------------------------------
