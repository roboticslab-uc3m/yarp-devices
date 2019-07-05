// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <ColorDebug.h>

// ------------------- IPositionDirect Related ------------------------------------

bool roboticslab::DextraControlboardUSB::setPosition(int j, double ref)
{
    CD_DEBUG("(%d, %f)\n", j, ref);
    return positionMove(j, ref);
}

// ----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setPositions(const int n_joint, const int *joints, const double *refs)
{
    CD_DEBUG("(%d)\n", n_joint);
    return positionMove(n_joint, joints, refs);
}

// ----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setPositions(const double *refs)
{
    CD_DEBUG("\n");
    return positionMove(refs);
}

// ----------------------------------------------------------------------------------------
