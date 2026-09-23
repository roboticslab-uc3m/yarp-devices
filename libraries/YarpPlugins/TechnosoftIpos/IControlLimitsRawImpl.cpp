// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::setPosLimitsRaw(int axis, double min, double max)
#else
bool TechnosoftIposBase::setLimitsRaw(int axis, double min, double max)
#endif
{
    CHECK_JOINT(axis);

    bool okMin = false;
    bool okMax = false;

    if (setPosLimitRaw(min, true))
    {
        this->min = min;
        okMin = true;
    }

    if (setPosLimitRaw(max, false))
    {
        this->max = max;
        okMax = true;
    }

    return okMin && okMax
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ? yarp::dev::ReturnValue::return_code::return_value_ok : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        ;
#endif
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::setPosLimitRaw(double limit, bool isMin)
{
    std::string name = "Software position limit: ";
    std::uint8_t subindex;

    if (isMin ^ params.m_reverse)
    {
        name += "minimal position limit";
        subindex = 0x01;
    }
    else
    {
        name += "maximal position limit";
        subindex = 0x02;
    }

    std::int32_t data = degreesToInternalUnits(limit);
    return can->sdo()->download(name, data, 0x607D, subindex);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getPosLimitsRaw(int axis, double * min, double * max)
#else
bool TechnosoftIposBase::getLimitsRaw(int axis, double * min, double * max)
#endif
{
    CHECK_JOINT(axis);

    if (actualControlMode == VOCAB_CM_NOT_CONFIGURED)
    {
        *min = this->min;
        *max = this->max;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
        return true;
#endif
    }

    return getPosLimitRaw(min, true) & getPosLimitRaw(max, false)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ? yarp::dev::ReturnValue::return_code::return_value_ok : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        ;
#endif
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getPosLimitRaw(double * limit, bool isMin)
{
    std::string name = "Software position limit: ";
    std::uint8_t subindex;

    if (isMin ^ params.m_reverse)
    {
        name += "minimal position limit";
        subindex = 0x01;
    }
    else
    {
        name += "maximal position limit";
        subindex = 0x02;
    }

    return can->sdo()->upload<std::int32_t>(name, [this, limit](auto data)
        { *limit = internalUnitsToDegrees(data); },
        0x607D, subindex);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::setVelLimitsRaw(int axis, double min, double max)
#else
bool TechnosoftIposBase::setVelLimitsRaw(int axis, double min, double max)
#endif
{
    CHECK_JOINT(axis);

    maxVel = max;

    if (min != -max)
    {
        yCIWarning(IPOS, id()) << "Minimum value not equal to negative maximum value";
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}
// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getVelLimitsRaw(int axis, double * min, double * max)
#else
bool TechnosoftIposBase::getVelLimitsRaw(int axis, double * min, double * max)
#endif
{
    CHECK_JOINT(axis);

    *min = -maxVel;
    *max = maxVel;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
