// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

#include <cmath>
#include <cstring>

#include <ColorDebug.h>

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
    CD_DEBUG("(%d, %f)\n", m, ref);
    CHECK_JOINT(m);

    const std::size_t len = sizeof ref;
    ref = std::max(100.0, std::min(ref, -100.0));
    std::uint8_t msgData[len];
    std::memcpy(msgData, &ref, len);

    return send(len, msgData);
}

//--------------------------------------------------------------------------------------

bool LacqueyFetch::setRefDutyCyclesRaw(const double * refs)
{
    CD_DEBUG("\n");
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
    CD_DEBUG("\n");
    return getRefDutyCycleRaw(0, &refs[0]);
}

//--------------------------------------------------------------------------------------

bool LacqueyFetch::getDutyCycleRaw(int m, double * val)
{
    CHECK_JOINT(m);
    CD_WARNING("Not supported.\n");
    return false;
}

//--------------------------------------------------------------------------------------

bool LacqueyFetch::getDutyCyclesRaw(double * vals)
{
    CD_WARNING("Not supported.\n");
    return false;
}

//--------------------------------------------------------------------------------------
