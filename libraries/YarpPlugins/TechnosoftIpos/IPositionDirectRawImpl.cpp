// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setPositionRaw(int j, double ref)
{
    CD_DEBUG("(%d, %f)\n", j, ref);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION_DIRECT);
    return quitHaltState(VOCAB_CM_POSITION_DIRECT) && (vars.synchronousCommandTarget = ref, true);
}
// -----------------------------------------------------------------------------

bool TechnosoftIpos::setPositionsRaw(const double * refs)
{
    CD_DEBUG("\n");
    return setPositionRaw(0, refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setPositionsRaw(int n_joint, const int * joints, const double * refs)
{
    CD_DEBUG("\n");
    return setPositionRaw(joints[0], refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefPositionRaw(int joint, double * ref)
{
    CD_DEBUG("(%d)\n", joint);
    CHECK_JOINT(joint);
    CHECK_MODE(VOCAB_CM_POSITION_DIRECT);
    *ref = vars.synchronousCommandTarget;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefPositionsRaw(double * refs)
{
    CD_DEBUG("\n");
    return getRefPositionRaw(0, &refs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getRefPositionsRaw(int n_joint, const int * joints, double * refs)
{
    CD_DEBUG("\n");
    return getRefPositionRaw(joints[0], &refs[0]);
}

// -----------------------------------------------------------------------------
