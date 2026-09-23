// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getCurrentRaw(int m, double * curr)
#else
bool TechnosoftIposBase::getCurrentRaw(int m, double * curr)
#endif
{
    CHECK_JOINT(m);
    std::int16_t temp = lastCurrentRead;
    *curr = internalUnitsToCurrent(temp);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getCurrentRangeRaw(int m, double * min, double * max)
#else
bool TechnosoftIposBase::getCurrentRangeRaw(int m, double * min, double * max)
#endif
{
    CHECK_JOINT(m);

    return can->sdo()->upload<std::uint16_t>("Current limit", [this, min, max](auto data)
        { *max = internalUnitsToPeakCurrent(data);
          *min = -(*max); },
        0x207F)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
        ;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::setRefCurrentRaw(int m, double curr)
#else
bool TechnosoftIposBase::setRefCurrentRaw(int m, double curr)
#endif
{
    CHECK_JOINT(m);
    CHECK_MODE(VOCAB_CM_CURRENT);

    const auto state = limitSwitchState.load();

    if (state == INACTIVE || (state == POSITIVE && curr <= 0.0) || (state == NEGATIVE && curr >= 0.0))
    {
        commandBuffer.accept(curr);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_ok;
#else
        return true;
#endif
    }
    else
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getRefCurrentRaw(int m, double * curr)
#else
bool TechnosoftIposBase::getRefCurrentRaw(int m, double * curr)
#endif
{
    CHECK_JOINT(m);
    CHECK_MODE(VOCAB_CM_CURRENT);
    *curr = commandBuffer.getStoredCommand();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
