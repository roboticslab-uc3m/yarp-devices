// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::open(yarp::os::Searchable & config)
{
    if (!setRemoteVariableRaw("enableCsv", {yarp::os::Value(params.m_enableCsv)}))
    {
        return false;
    }

    if (params.m_initialInteractionMode != yarp::os::Vocab32::decode(yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT) &&
        params.m_initialInteractionMode != yarp::os::Vocab32::decode(yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT) &&
        params.m_initialInteractionMode != yarp::os::Vocab32::decode(yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT))
    {
        yCIError(IPOS, id()) << "Illegal interaction mode vocab:" << params.m_initialInteractionMode;
        return false;
    }

    initialInteractionMode = yarp::os::Vocab32::encode(params.m_initialInteractionMode);

    if (params.m_stiffness < params.m_minStiffness || params.m_stiffness > params.m_maxStiffness)
    {
        yCIError(IPOS, id(), "Invalid stiffness: %f (not in [%f, %f])", params.m_stiffness, params.m_minStiffness, params.m_maxStiffness);
        return false;
    }

    if (params.m_damping < params.m_minDamping || params.m_damping > params.m_maxDamping)
    {
        yCIError(IPOS, id(), "Invalid damping: %f (not in [%f, %f])", params.m_damping, params.m_minDamping, params.m_maxDamping);
        return false;
    }

    if (params.m_impedanceOffset < 0.0)
    {
        yCIError(IPOS, id()) << "Illegal offset:" << params.m_impedanceOffset;
        return false;
    }

    positionPid.setKp(params.m_kp);
    positionPid.setKi(params.m_ki);
    positionPid.setKd(params.m_kd);
    positionPid.setMaxInt(params.m_maxInt);
    positionPid.setMaxOut(params.m_maxOutput);
    positionPid.setOffset(params.m_offset);
    positionPid.setScale(params.m_scale);
    positionPid.setStictionValues(params.m_stictionUp, params.m_stictionDown);
    positionPid.setKff(params.m_kff);

    impedancePid.setKp(params.m_stiffness);
    impedancePid.setKd(params.m_damping);
    impedancePid.setMaxOut(params.m_maxOutput); // re-use
    impedancePid.setOffset(params.m_impedanceOffset);
    impedancePid.setScale(1.0);
    impedancePid.setStictionValues(params.m_stictionUp, params.m_stictionDown); // re-use

    if (params.m_errorLimit <= 0.0)
    {
        yCIError(IPOS, id()) << "Illegal position error limit:" << params.m_errorLimit;
        return false;
    }

    errorLimit = params.m_errorLimit;

    return TechnosoftIposBase::open(config);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::close()
{
    return TechnosoftIposBase::close();
}

// -----------------------------------------------------------------------------
