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
    //CD_WARNING("Not supported.\n"); // too verbose in controlboardwrapper2 stream
    return false;
}

//--------------------------------------------------------------------------------------
