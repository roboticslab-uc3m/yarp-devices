// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

using namespace roboticslab;

// -------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getRefTorqueRaw(int j, double * t)
#else
bool TechnosoftIposBase::getRefTorqueRaw(int j, double * t)
#endif
{
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_TORQUE);
    *t = commandBuffer.getStoredCommand();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::setRefTorqueRaw(int j, double t)
#else
bool TechnosoftIposBase::setRefTorqueRaw(int j, double t)
#endif
{
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_TORQUE);

    const auto state = limitSwitchState.load();

    if (state == INACTIVE || (state == POSITIVE && t <= 0.0) || (state == NEGATIVE && t >= 0.0))
    {
        commandBuffer.accept(t);
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

// -------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getTorqueRaw(int j, double * t)
#else
bool TechnosoftIposBase::getTorqueRaw(int j, double * t)
#endif
{
    CHECK_JOINT(j);
    std::int16_t temp = lastCurrentRead;
    double curr = internalUnitsToCurrent(temp);
    *t = currentToTorque(curr);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getTorqueRangeRaw(int j, double * min, double * max)
#else
bool TechnosoftIposBase::getTorqueRangeRaw(int j, double * min, double * max)
#endif
{
    CHECK_JOINT(j);

    return can->sdo()->upload<std::uint16_t>("Current limit", [this, min, max](auto data)
        { double temp = internalUnitsToPeakCurrent(data);
          *max = currentToTorque(temp);
          *min = -(*max); },
        0x207F)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
        ;
#endif
}

// -------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params)
#else
bool TechnosoftIposBase::getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params)
#endif
{
    CHECK_JOINT(j);

    params->bemf = 0.0;
    params->bemf_scale = 0.0;
    params->ktau = k;
    params->ktau_scale = 0.0;
    params->viscousPos = 0.0;
    params->viscousNeg = 0.0;
    params->coulombPos = 0.0;
    params->coulombNeg = 0.0;
    params->velocityThres = 0.0;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -------------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params)
#else
bool TechnosoftIposBase::setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params)
#endif
{
    CHECK_JOINT(j);
    k = params.ktau;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -------------------------------------------------------------------------------------
