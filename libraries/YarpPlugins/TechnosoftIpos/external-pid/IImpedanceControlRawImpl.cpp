// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getImpedanceRaw(int j, double * stiffness, double * damping)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setImpedanceRaw(int j, double stiffness, double damping)
{
    yCITrace(IPOS, id(), "%d %f %f", j, stiffness, damping);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setImpedanceOffsetRaw(int j, double offset)
{
    yCITrace(IPOS, id(), "%d %f", j, offset);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getImpedanceOffsetRaw(int j, double * offset)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getCurrentImpedanceLimitRaw(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------
