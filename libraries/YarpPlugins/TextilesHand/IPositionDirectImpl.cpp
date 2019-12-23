// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// ############################## IPositionDirectRaw Related ##############################

bool roboticslab::TextilesHand::setPosition(int j, double ref)
{
    CD_DEBUG("\n");
    this->positionMove(0,ref);
    return true;
}

// ----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setPositions(const int n_joint, const int *joints, const double *refs)
{
    CD_DEBUG("\n");
    this->positionMove(0,refs[0]);
    return true;
}

// ----------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setPositions(const double *refs)
{
    CD_DEBUG("\n");
    return true;
}

// ----------------------------------------------------------------------------------------
