// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setPositionRaw(int j, double ref)
{
    yCITrace(IPOS, id(), "%d %f", j, ref);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION_DIRECT);
    vars.synchronousCommandTarget = ref;
    return true;
}
// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setPositionsRaw(const double * refs)
{
    return setPositionRaw(0, refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::setPositionsRaw(int n_joint, const int * joints, const double * refs)
{
    return setPositionRaw(joints[0], refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefPositionRaw(int joint, double * ref)
{
    yCITrace(IPOS, id(), "%d", joint);
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_POSITION_DIRECT);
    *ref = vars.synchronousCommandTarget;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefPositionsRaw(double * refs)
{
    return getRefPositionRaw(0, &refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposExternal::getRefPositionsRaw(int n_joint, const int * joints, double * refs)
{
    return getRefPositionRaw(joints[0], &refs[0]);
}

// -----------------------------------------------------------------------------
