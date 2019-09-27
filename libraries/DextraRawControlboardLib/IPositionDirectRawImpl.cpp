// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// ----------------------------------------------------------------------------------------

bool DextraRawControlboard::setPositionRaw(int j, double ref)
{
    CD_DEBUG("(%d, %f)\n", j, ref);
    return positionMoveRaw(j, ref);
}

// ----------------------------------------------------------------------------------------

bool DextraRawControlboard::setPositionsRaw(const double * refs)
{
    CD_DEBUG("\n");
    return positionMoveRaw(refs);
}

// ----------------------------------------------------------------------------------------

bool DextraRawControlboard::setPositionsRaw(int n_joint, const int * joints, const double * refs)
{
    CD_DEBUG("\n");
    return positionMoveRaw(n_joint, joints, refs);
}

// ----------------------------------------------------------------------------------------
