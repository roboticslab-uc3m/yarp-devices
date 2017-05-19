// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// ############################## IPositionDirectRaw Related ##############################

bool teo::TextilesHand::setPositionRaw(int j, double ref)
{
    CD_DEBUG("\n");
    this->positionMoveRaw(0,ref);
    return true;
}

// ----------------------------------------------------------------------------------------

bool teo::TextilesHand::setPositionsRaw(const int n_joint, const int *joints, double *refs)
{
    CD_DEBUG("\n");
    this->positionMoveRaw(0,refs[0]);
    return true;
}

// ----------------------------------------------------------------------------------------

bool teo::TextilesHand::setPositionsRaw(const double *refs)
{
    CD_DEBUG("\n");
    return true;
}

// ----------------------------------------------------------------------------------------
