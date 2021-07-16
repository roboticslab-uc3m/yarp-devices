// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <cmath>
#include <cstring>

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

//--------------------------------------------------------------------------------------

bool LacqueyFetch::getNumberOfMotorsRaw(int * number)
{
    *number = 1;
    return true;
}

//--------------------------------------------------------------------------------------

bool LacqueyFetch::setRefDutyCycleRaw(int m, double ref)
{
    yCTrace(LCQ, "%d %f", m, ref);
    CHECK_JOINT(m);

    // clip between -100% and +100%
    ref = std::min(100.0, std::max(ref, -100.0));

    const std::size_t len = sizeof refDutyCycles;
    std::uint8_t msgData[len];
    refDutyCycles = ref;
    std::memcpy(msgData, &refDutyCycles, len);

    return send(len, msgData);
}

//--------------------------------------------------------------------------------------

bool LacqueyFetch::setRefDutyCyclesRaw(const double * refs)
{
    return setRefDutyCycleRaw(0, refs[0]);
}

//--------------------------------------------------------------------------------------

bool LacqueyFetch::getRefDutyCycleRaw(int m, double * ref)
{
    CHECK_JOINT(m);
    *ref = refDutyCycles;
    return true;
}

//--------------------------------------------------------------------------------------

bool LacqueyFetch::getRefDutyCyclesRaw(double * refs)
{
    return getRefDutyCycleRaw(0, &refs[0]);
}

//--------------------------------------------------------------------------------------

bool LacqueyFetch::getDutyCycleRaw(int m, double * val)
{
    CHECK_JOINT(m);
    yCWarning(LCQ, "getDutyCycleRaw() not supported");
    return false;
}

//--------------------------------------------------------------------------------------

bool LacqueyFetch::getDutyCyclesRaw(double * vals)
{
    return false;
}

//--------------------------------------------------------------------------------------
