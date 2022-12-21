// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <cmath> // std::abs

#include <algorithm> // std::clamp

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

// only position PID (for now?)
#define CHECK_PID_TYPE(type) do { if ((type) != yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION) return false; } while (0)

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, const yarp::dev::Pid & pid)
{
    yCITrace(IPOS, id(), "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    positionPid = pid;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const yarp::dev::Pid * pids)
{
    return setPidRaw(pidtype, 0, pids[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double ref)
{
    yCITrace(IPOS, id(), "%s %d %f", yarp::os::Vocab32::decode(pidtype).c_str(), j, ref);
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    positionReference = ref;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * refs)
{
    return setPidReferenceRaw(pidtype, 0, refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double limit)
{
    yCITrace(IPOS, id(), "%s %d %f", yarp::os::Vocab32::decode(pidtype).c_str(), j, limit);
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    positionErrorLimit = limit;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, const double * limits)
{
    return setPidErrorLimitRaw(pidtype, 0, limits[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getPidErrorRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * err)
{
    yCITrace(IPOS, id(), "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    *err = positionReference - internalUnitsToDegrees(lastEncoderRead->queryPosition());
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getPidErrorsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * errs)
{
    return getPidErrorRaw(pidtype, 0, &errs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getPidOutputRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * out)
{
    yCITrace(IPOS, id(), "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);

    double position = internalUnitsToDegrees(lastEncoderRead->queryPosition());
    double scale = 1.0 / (activePid->scale * activePid->scale);

    std::lock_guard lock(pidMutex);
    double prevProportionalError = proportionalError;
    proportionalError = position - positionReference;

    if (std::abs(proportionalError) > positionErrorLimit)
    {
        yCIError(IPOS, id(), "Proportional error %f exceeds limit %f", std::abs(proportionalError), positionErrorLimit);
        return false;
    }

    double proportionalTerm = activePid->kp * proportionalError * scale;

    double integralTerm = 0.0;

    if (activePid->ki != 0.0)
    {
        integralError += proportionalError * syncPeriod;
        integralTerm = std::clamp(activePid->ki * integralError * scale, -activePid->max_int, activePid->max_int);
        integralError = integralTerm / activePid->ki;
    }

    double derivativeError = (proportionalError - prevProportionalError) / syncPeriod;
    double derivativeTerm = activePid->kd * derivativeError * scale;

    double feedForwardTerm = positionReference * activePid->kff;
    double combinedTerms = proportionalTerm + integralTerm + derivativeTerm + feedForwardTerm + activePid->offset;

    *out = std::clamp(combinedTerms, -activePid->max_output, activePid->max_output);
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getPidOutputsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * outs)
{
    return getPidOutputRaw(pidtype, 0, &outs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, yarp::dev::Pid * pid)
{
    yCITrace(IPOS, id(), "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    *pid = positionPid;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getPidsRaw(const yarp::dev::PidControlTypeEnum & pidtype, yarp::dev::Pid * pids)
{
    return getPidRaw(pidtype, 0, &pids[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getPidReferenceRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * ref)
{
    yCITrace(IPOS, id(), "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    *ref = positionReference;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getPidReferencesRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * refs)
{
    return getPidReferenceRaw(pidtype, 0, &refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getPidErrorLimitRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double * limit)
{
    yCITrace(IPOS, id(), "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    *limit = positionErrorLimit;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getPidErrorLimitsRaw(const yarp::dev::PidControlTypeEnum & pidtype, double * limits)
{
    return getPidErrorLimitRaw(pidtype, 0, &limits[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::resetPidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    yCITrace(IPOS, id(), "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    positionReference = internalUnitsToDegrees(lastEncoderRead->queryPosition());
    proportionalError = integralError = 0.0;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::disablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    yCITrace(IPOS, id(), "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    yCIError(IPOS, id(), "disablePidRaw() not implemented");
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::enablePidRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j)
{
    yCITrace(IPOS, id(), "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    yCIError(IPOS, id(), "enablePidRaw() not implemented");
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setPidOffsetRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, double v)
{
    yCITrace(IPOS, id(), "%s %d %f", yarp::os::Vocab32::decode(pidtype).c_str(), j, v);
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    std::lock_guard lock(pidMutex);
    activePid->offset = v;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::isPidEnabledRaw(const yarp::dev::PidControlTypeEnum & pidtype, int j, bool * enabled)
{
    yCITrace(IPOS, id(), "%s %d", yarp::os::Vocab32::decode(pidtype).c_str(), j);
    CHECK_JOINT(j);
    CHECK_PID_TYPE(pidtype);
    return true;
}

// -----------------------------------------------------------------------------
