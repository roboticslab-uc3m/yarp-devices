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

    if (stiffness < minStiffness || stiffness > maxStiffness)
    {
        yCIError(IPOS, id(), "Invalid stiffness: %f (not in [%f, %f])", stiffness, minStiffness, maxStiffness);
        return false;
    }

    if (damping < minDamping || damping > maxDamping)
    {
        yCIError(IPOS, id(), "Invalid damping: %f (not in [%f, %f])", damping, minDamping, maxDamping);
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

    *min_stiff = minStiffness;
    *max_stiff = maxStiffness;
    *min_damp = minDamping;
    *max_damp = maxDamping;

    return true;
}

// -----------------------------------------------------------------------------
