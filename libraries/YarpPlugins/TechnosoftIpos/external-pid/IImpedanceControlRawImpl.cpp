// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getImpedanceRaw(int j, double * stiffness, double * damping)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    std::lock_guard lock(pidMutex);
    *stiffness = impedancePid.kp;
    *damping = impedancePid.kd;

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setImpedanceRaw(int j, double stiffness, double damping)
{
    yCITrace(IPOS, id(), "%d %f %f", j, stiffness, damping);
    CHECK_JOINT(j);

    if (stiffness < params.m_minStiffness || stiffness > params.m_maxStiffness)
    {
        yCIError(IPOS, id(), "Invalid stiffness: %f (not in [%f, %f])", stiffness, params.m_minStiffness, params.m_maxStiffness);
        return false;
    }

    if (damping < params.m_minDamping || damping > params.m_maxDamping)
    {
        yCIError(IPOS, id(), "Invalid damping: %f (not in [%f, %f])", damping, params.m_minDamping, params.m_maxDamping);
        return false;
    }

    std::lock_guard lock(pidMutex);
    impedancePid.setKp(stiffness);
    impedancePid.setKd(damping);
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setImpedanceOffsetRaw(int j, double offset)
{
    yCITrace(IPOS, id(), "%d %f", j, offset);
    CHECK_JOINT(j);

    std::lock_guard lock(pidMutex);
    impedancePid.setOffset(offset);

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getImpedanceOffsetRaw(int j, double * offset)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    std::lock_guard lock(pidMutex);
    *offset = impedancePid.offset;

    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getCurrentImpedanceLimitRaw(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    *min_stiff = params.m_minStiffness;
    *max_stiff = params.m_maxStiffness;
    *min_damp = params.m_minDamping;
    *max_damp = params.m_maxDamping;

    return true;
}

// -----------------------------------------------------------------------------
