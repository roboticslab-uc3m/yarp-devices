// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <cmath> // std::abs

#include <algorithm> // std::clamp

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

// only position PID (for now?)
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
#define CHECK_PID_TYPE(type) do { if ((type) != yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION) return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device; } while (0)
#else
#define CHECK_PID_TYPE(type) do { if ((type) != yarp::dev::VOCAB_PIDTYPE_POSITION) return false; } while (0)
#endif

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getAvailablePidsRaw(int j, std::vector<yarp::dev::PidControlTypeEnum> & types)
{
    CHECK_JOINT(j);
    types = {yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION};
    return yarp::dev::ReturnValue::return_code::return_value_ok;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::setPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid)
#else
bool TechnosoftIposExternal::setPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid)
#endif
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    positionPid = pid;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v)
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    yCIError(IPOS, id(), "getPidOffsetRaw() not implemented");
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue TechnosoftIposExternal::getPidFeedforwardRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double & v)
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    yCIError(IPOS, id(), "getPidFeedforwardRaw() not implemented");
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue TechnosoftIposExternal::getPidExtraInfoRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::PidExtraInfo & info)
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    yCIError(IPOS, id(), "getPidExtraInfoRaw() not implemented");
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue TechnosoftIposExternal::getPidExtraInfosRaw(const yarp::dev::PidControlTypeEnum & pidtype, std::vector<yarp::dev::PidExtraInfo> & info)
{
    CHECK_PID_TYPE(pidtype);
    yCIError(IPOS, id(), "getPidExtraInfosRaw() not implemented");
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::setPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref)
#else
bool TechnosoftIposExternal::setPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref)
#endif
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    positionReference = ref;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::setPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit)
#else
bool TechnosoftIposExternal::setPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit)
#endif
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    errorLimit = limit;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getPidErrorRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err)
#else
bool TechnosoftIposExternal::getPidErrorRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err)
#endif
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    *err = proportionalError;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getPidOutputRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out)
#else
bool TechnosoftIposExternal::getPidOutputRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out)
#endif
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);

    double position = internalUnitsToDegrees(lastEncoderRead->queryPosition());
    double scale = 1.0 / (activePid->scale * activePid->scale);

    std::lock_guard lock(pidMutex);
    double prevProportionalError = proportionalError;
    proportionalError = positionReference - position;

    if (std::abs(proportionalError) > errorLimit)
    {
        yCIWarning(IPOS, id(), "Proportional error %f clamped to %f", std::abs(proportionalError), errorLimit);
        proportionalError = std::clamp(proportionalError, -errorLimit, errorLimit);
    }

    double proportionalTerm = activePid->kp * proportionalError * scale;

    double integralTerm = 0.0;

    if (activePid->ki != 0.0)
    {
        integralError += proportionalError * params.m_syncPeriod;
        integralTerm = std::clamp(activePid->ki * integralError * scale, -activePid->max_int, activePid->max_int);
        integralError = integralTerm / activePid->ki;
    }

    double derivativeError = (proportionalError - prevProportionalError) / params.m_syncPeriod;
    double derivativeTerm = activePid->kd * derivativeError * scale;

    double feedForwardTerm = positionReference * activePid->kff;
    double combinedTerms = proportionalTerm + integralTerm + derivativeTerm + feedForwardTerm + activePid->offset;

    if (std::abs(combinedTerms) > activePid->max_output)
    {
        yCIWarning(IPOS, id(), "PID output %f clamped to %f", std::abs(combinedTerms), activePid->max_output);
        combinedTerms = std::clamp(combinedTerms, -activePid->max_output, activePid->max_output);
    }

    *out = combinedTerms;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid)
#else
bool TechnosoftIposExternal::getPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid)
#endif
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    *pid = positionPid;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref)
#else
bool TechnosoftIposExternal::getPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref)
#endif
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    *ref = positionReference;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit)
#else
bool TechnosoftIposExternal::getPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit)
#endif
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    *limit = errorLimit;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::resetPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j)
#else
bool TechnosoftIposExternal::resetPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j)
#endif
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    positionReference = internalUnitsToDegrees(lastEncoderRead->queryPosition());
    proportionalError = integralError = 0.0;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::disablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j)
#else
bool TechnosoftIposExternal::disablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j)
#endif
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    yCIError(IPOS, id(), "disablePidRaw() not implemented");
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::enablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j)
#else
bool TechnosoftIposExternal::enablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j)
#endif
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    yCIError(IPOS, id(), "enablePidRaw() not implemented");
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::setPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v)
#else
bool TechnosoftIposExternal::setPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v)
#endif
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    positionPid.offset = v;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::setPidFeedforwardRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v)
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    yCIError(IPOS, id(), "setPidFeedforwardRaw() not implemented");
    return yarp::dev::ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

// -----------------------------------------------------------------------------

yarp::dev::ReturnValue TechnosoftIposExternal::isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool & enabled)
#else
bool TechnosoftIposExternal::isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled)
#endif
{
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    enabled = true;
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    *enabled = true;
    return true;
#endif
}

// -----------------------------------------------------------------------------
