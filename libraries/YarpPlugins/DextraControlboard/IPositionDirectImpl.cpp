// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboard.hpp"

// ############################## IPositionDirect Related ##############################

bool roboticslab::DextraControlboard::setPosition(int j, double ref)
{
    CD_DEBUG("\n");
    this->positionMove(0,ref);
    return true;
}

// ----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setPositions(const int n_joint, const int *joints, const double *refs)
{
    CD_DEBUG("\n");
    this->positionMove(0,refs[0]);
    return true;
}

// ----------------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setPositions(const double *refs)
{
    CD_DEBUG("\n");
    return true;
}

// ----------------------------------------------------------------------------------------
