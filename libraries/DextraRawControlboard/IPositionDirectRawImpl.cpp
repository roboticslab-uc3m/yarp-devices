// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <ColorDebug.h>

// ------------------- IPositionDirect Related ------------------------------------

bool roboticslab::DextraRawControlboard::setPositionRaw(int j, double ref)
{
    CD_DEBUG("(%d, %f)\n", j, ref);
    return positionMoveRaw(j, ref);
}

// ----------------------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::setPositionsRaw(const int n_joint, const int *joints, const double *refs)
{
    CD_DEBUG("(%d)\n", n_joint);
    return positionMoveRaw(n_joint, joints, refs);
}

// ----------------------------------------------------------------------------------------

bool roboticslab::DextraRawControlboard::setPositionsRaw(const double *refs)
{
    CD_DEBUG("\n");
    return positionMoveRaw(refs);
}

// ----------------------------------------------------------------------------------------
