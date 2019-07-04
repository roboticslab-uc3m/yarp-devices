// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <ColorDebug.h>

// ############################## IPositionDirect Related ##############################

bool roboticslab::DextraControlboardUSB::setPosition(int j, double ref)
{
    CD_DEBUG("\n");
    this->positionMove(0,ref);
    return true;
}

// ----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setPositions(const int n_joint, const int *joints, const double *refs)
{
    CD_DEBUG("\n");
    this->positionMove(0,refs[0]);
    return true;
}

// ----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setPositions(const double *refs)
{
    CD_DEBUG("\n");
    return true;
}

// ----------------------------------------------------------------------------------------
