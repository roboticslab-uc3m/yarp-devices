// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getNumberOfMotorEncodersRaw(int * num)
{
    std::size_t ax;
    auto ret = getAxes(ax);
    *num = static_cast<int>(ax);
    return ret;
}
#else
bool TechnosoftIposBase::getNumberOfMotorEncodersRaw(int * num)
{
    return getAxes(num);
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::resetMotorEncoderRaw(int m)
#else
bool TechnosoftIposBase::resetMotorEncoderRaw(int m)
#endif
{
    CHECK_JOINT(m);
    return setMotorEncoderRaw(m, 0);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::setMotorEncoderCountsPerRevolutionRaw(int m, double cpr)
#else
bool TechnosoftIposBase::setMotorEncoderCountsPerRevolutionRaw(int m, double cpr)
#endif
{
    CHECK_JOINT(m);
    encoderPulses = cpr;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getMotorEncoderCountsPerRevolutionRaw(int m, double * cpr)
#else
bool TechnosoftIposBase::getMotorEncoderCountsPerRevolutionRaw(int m, double * cpr)
#endif
{
    CHECK_JOINT(m);
    *cpr = encoderPulses;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::setMotorEncoderRaw(int m, double val)
#else
bool TechnosoftIposBase::setMotorEncoderRaw(int m, double val)
#endif
{
    CHECK_JOINT(m);
    std::int32_t data = params.m_reverse ? -val : val;

    if (!can->sdo()->download("Set actual position", data, 0x2081))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
    }

    lastEncoderRead->reset(data);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getMotorEncoderRaw(int m, double * v)
#else
bool TechnosoftIposBase::getMotorEncoderRaw(int m, double * v)
#endif
{
    CHECK_JOINT(m);
    std::int32_t temp = lastEncoderRead->queryPosition();
    *v = params.m_reverse ? -temp : temp;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getMotorEncoderTimedRaw(int m, double * enc, double * stamp)
#else
bool TechnosoftIposBase::getMotorEncoderTimedRaw(int m, double * enc, double * stamp)
#endif
{
    CHECK_JOINT(m);
    std::int32_t temp =  lastEncoderRead->queryPosition();
    *enc = params.m_reverse ? -temp : temp;
    *stamp = lastEncoderRead->queryTime();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getMotorEncoderSpeedRaw(int m, double * sp)
#else
bool TechnosoftIposBase::getMotorEncoderSpeedRaw(int m, double * sp)
#endif
{
    CHECK_JOINT(m);
    double temp = lastEncoderRead->querySpeed();
    *sp = params.m_reverse ? -temp : temp;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getMotorEncoderAccelerationRaw(int m, double * acc)
#else
bool TechnosoftIposBase::getMotorEncoderAccelerationRaw(int m, double * acc)
#endif
{
    CHECK_JOINT(m);
    double temp = lastEncoderRead->queryAcceleration();
    *acc = params.m_reverse ? -temp : temp;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
