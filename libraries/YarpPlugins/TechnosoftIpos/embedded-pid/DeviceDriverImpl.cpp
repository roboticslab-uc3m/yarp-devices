// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "embedded-pid/TechnosoftIposEmbedded.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposEmbedded::open(yarp::os::Searchable & config)
{
    if (!setRemoteVariableRaw("ipMode", {yarp::os::Value(params.m_ipMode)}) ||
        !setRemoteVariableRaw("ipPeriodMs", {yarp::os::Value(params.m_ipPeriodMs)}) ||
        !setRemoteVariableRaw("enableIp", {yarp::os::Value(params.m_enableIp)}) ||
        !setRemoteVariableRaw("enableCsv", {yarp::os::Value(params.m_enableCsv)}))
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
