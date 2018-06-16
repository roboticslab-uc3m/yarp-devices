// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

//  ########################### IVelocityControlRaw implementations ###########################

bool roboticslab::TextilesHand::velocityMoveRaw(int j, double sp)
{
    CD_INFO("Missing implementation\n");
    return true;
}

// --------------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::velocityMoveRaw(const double *sp)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//  ########################### IVelocityControl2Raw implementations ###########################

bool roboticslab::TextilesHand::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getRefVelocityRaw(const int joint, double *vel)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getRefVelocitiesRaw(double *vels)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getRefVelocitiesRaw(const int n_joint, const int *joints, double *vels)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// --------------------------------------------------------------------------------------------
